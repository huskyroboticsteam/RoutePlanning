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
    turnstate = TOWARD_TARGET;
    turning = true;
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
            if (angleCloseEnough(agent.getInternalRotation(), tar_angle, 0.5))
            {
                // printf("reached turning target of %f\n", tar_angle);
                orig_angle = agent.getInternalRotation();
                tar_angle = orig_angle + AUTO_TURN_RANGE / 2.f;
#if SURVEY
turnstate = SURVEY_COUNTERCW;
#else
                turning = false;
                timer.reset();
#endif
                break;
            }
            // printf("turning toward target %f\n", tar_angle);
            grid.rotateAgent(agent, agent.turnTowards(tar_angle));
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
                turning = false;
                timer.reset();
                break;
            }
            grid.rotateAgent(agent, agent.turnTowards(tar_angle));
            break;
        }
    }
    else
    {
        std::vector<point> path = map.shortest_path_to();
        if (timer.elapsed() > AUTO_MOVE_TIME)
        {
            // turning time 
            // prevent robot from getting stuck at one point
            std::vector<point>::iterator next = path.begin();
            // for (next = path.begin(); next != path.end() &&
            //     same_point(*next, point{agent.getX(), agent.getY()}, 0.3f); next++) {}
            tar_angle = atan2(next->y - agent.getY(),
                              next->x - agent.getX()) *
                        180 / PI;
            if (path.size() == 1)
            {
                // if no obstacle, go straight to point
                turning = true;
                turnstate = BACK_TO_TARGET;
            }
            else
            {
                init_turn();
            }
        }
        else
        {
            // moving time
            float dist = sqrt(dist_sq(path.front(), point{agent.getX(), agent.getY()}));
            float speed;
            if (path.size() == 1)
            {
                speed = 0.12f * (dist - 1.f);
            }
            else
            {
                speed = 0.11f * dist + 0.2f;
            }

            if (speed > 1.f)
                speed = 1.f;
            grid.moveAgent(agent, agent.drive(speed));
        }
    }
}

void RP::AutoController::stop_auto()
{
}