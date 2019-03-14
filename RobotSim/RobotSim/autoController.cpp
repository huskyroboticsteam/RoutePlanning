#include "autoController.hpp"
#include "utils.hpp"

#define SURVEY 0 // whether to look around before moving

void RP::AutoController::start_auto()
{
    // timer.reset();
    point target = map.compute_next_point();
    tar_angle = atan2(target.y - agent.getY(),
                      target.x - agent.getX()) *
                180 / PI;
    init_turn();
}

// TODO turn to next obstacle first and then survey
void RP::AutoController::init_turn()
{
    timer.reset();
    turnstate = TOWARD_TARGET;
    turning = true;
}

void RP::AutoController::turn_and_go()
{
    auto path = map.shortest_path_to();
    float dist = sqrt(dist_sq(path.front(), point{agent.getX(), agent.getY()}));
    if (path.size() == 1)
    {
        last_move_time = 0.5f;
        last_move_speed = 1.f;
    }
    else
    {
        // TODO tune speed
        last_move_time = 0.3f + 0.05f * dist;
        last_move_speed = std::min(0.2f + 0.1f * dist, 1.f);
        printf("time: %f, speed: %f, dist: %f\n", last_move_time, last_move_speed, dist);
    }
    turning = false;
    timer.reset();
}

void RP::AutoController::tic()
{
    if (turning)
    {
        // if (!turning_left)
        // {
        //     if (agent.getRotation() + AUTO_TURN_RANGE / 2 )
        // }
        // else
        // {
        //     auto path = map.shortest_path_to();
        //     RP::point st_target = path.front();
        //     grid.rotateAgent(agent, agent.turnTowards(st_target.x, st_target.y));
        // }
        switch (turnstate)
        {
        case TOWARD_TARGET:
            if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 1.))
            {
                auto path = map.shortest_path_to();
                if (path.size() == 0)
                    break;
                // TODO this is a hack. later, implement flag in map that tells if tar is the final target
                // or even better, implement a better algorithm
                if (path.size() == 1 && dist_sq(point{agent.getX(), agent.getY()}, path.back()) <= 3 * 3 && same_point(path.back(), map.tar))
                {
                    printf("Target reached.\n");
                    turnstate = FIND_BALL;
                    break;
                }
                tar_angle = compute_target_angle();
                if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 5.))
                {
                    // printf("reached turning target of %f\n", tar_angle);
                    // TODO recompute tar_angle again and iteratively turn until angle is the same as target angle
                    turn_and_go();
                }
                else
                {
                    timer.reset(); // turn again
                }
            }
            else
            {
                // printf("turning toward target %f\n", tar_angle);
                grid.rotateAgent(agent, agent.turnTowards(tar_angle));
            }

            break;
        case SURVEY_COUNTERCW:
            if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 0.5))
            {
                tar_angle = orig_angle - AUTO_TURN_RANGE / 2.f;
                turnstate = SURVEY_CW;
                // printf("turning cw toward %f\n", tar_angle);
                break;
            }
            grid.rotateAgent(agent, agent.turnTowards(tar_angle));
            break;
        case SURVEY_CW:
            if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 0.5))
            {
                auto tar_point = map.compute_next_point();
                // printf("target point: %f, %f\n", tar_point.x, tar_point.y);
                tar_angle = atan2(tar_point.y - agent.getY(), tar_point.x - agent.getX()) * 180 / PI;
                // printf("turning toward target %f\n", tar_angle);
                turnstate = BACK_TO_TARGET;
                break;
            }
            grid.rotateAgent(agent, agent.turnTowards(tar_angle));
            break;
        case BACK_TO_TARGET:
            if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 0.5))
            {
                // printf("reached turning target of %f\n", tar_angle);
                // TODO recompute tar_angle again and iteratively turn until angle is the same as target angle
                tar_angle = compute_target_angle();
                if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 0.5))
                {
                    printf("close enough to %f\n", tar_angle);
                    turn_and_go();
                }
                break;
            }
            grid.rotateAgent(agent, agent.turnTowards(tar_angle));
            break;
        case FIND_BALL:
            printf("WARNING: find tennis ball not implemented.\n");
            turnstate = FINISHED;
            break;
        case FINISHED:
            // TODO send back whatever instructions needed
            break;
        }
    }
    else
    {
        std::vector<point> path = map.shortest_path_to();
        if (path.empty())
            return;
        if (timer.elapsed() > last_move_time)
        {
            tar_angle = compute_target_angle();
            if (path.size() == 1)
            {
                // if no obstacle, go straight to point
                turning = true;
                turnstate = TOWARD_TARGET;
            }
            else
            {
                init_turn();
            }
        }
        else
        {
            grid.moveAgent(agent, agent.drive(last_move_speed));
        }
    }
}

float RP::AutoController::compute_target_angle()
{
    auto path = map.shortest_path_to();
    std::vector<point>::iterator next = path.begin();

    // prevent robot from getting stuck at one point
    for (next = path.begin(); next != path.end() &&
                              same_point(*next, point{agent.getX(), agent.getY()}, 1.f);
         next++);
    return atan2(next->y - agent.getY(), next->x - agent.getX()) * 180 / PI;
}

void RP::AutoController::stop_auto()
{
}