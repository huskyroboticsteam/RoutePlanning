#include "simController.hpp"
#include "autonomous/utils.hpp"

#define SURVEY 0 // whether to look around before moving
constexpr float TARGET_TOL = 4.f; // tolerance distance for testing if we've reached target
constexpr float TARGET_TOL_SQ = TARGET_TOL * TARGET_TOL;

void RP::SimController::start_auto()
{
    // timer.reset();
    point target = mapper.compute_next_point();
    tar_angle = atan2(target.y - agent.getY(),
                      target.x - agent.getX()) *
                180 / PI;
    init_turn();
}

// TODO turn to next obstacle first and then survey
void RP::SimController::init_turn()
{
    timer.reset();
    turnstate = TOWARD_TARGET;
    turning = true;
}

void RP::SimController::turn_and_go()
{
    auto path = mapper.compute_path();
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
        last_move_speed = std::min(0.2f + 0.05f * dist, 1.f);
        printf("time: %f, speed: %f, dist: %f\n", last_move_time, last_move_speed, dist);
    }
    turning = false;
    timer.reset();
}

void RP::SimController::tic()
{
    if (turning)
    {
        // if (!turning_left)
        // {
        //     if (agent.getRotation() + AUTO_TURN_RANGE / 2 )
        // }
        // else
        // {
        //     auto path = mapper.compute_path();
        //     RP::point st_target = path.front();
        //     grid.rotateAgent(agent, agent.turnTowards(st_target.x, st_target.y));
        // }
        switch (turnstate)
        {
        case TOWARD_TARGET:
            if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 1.))
            {
                auto path = mapper.compute_path();
                if (path.size() == 0)
                    break;
                // TODO this is a hack. later, implement flag in mapper that tells if tar is the final target
                // or even better, implement a better algorithm
                if (path.size() == 1 && dist_sq(point{agent.getX(), agent.getY()}, path.back()) <= TARGET_TOL_SQ && same_point(path.back(), mapper.tar))
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
                auto tar_point = mapper.compute_next_point();
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
        std::vector<point> path = mapper.compute_path();
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

float RP::SimController::compute_target_angle()
{
    auto path = mapper.compute_path();
    std::vector<point>::iterator next = path.begin();

    // prevent robot from getting stuck at one point
    for (next = path.begin(); next != path.end() &&
                              same_point(*next, point{agent.getX(), agent.getY()}, 1.f);
         next++);
    return atan2(next->y - agent.getY(), next->x - agent.getX()) * 180 / PI;
}

void RP::SimController::stop_auto()
{
}