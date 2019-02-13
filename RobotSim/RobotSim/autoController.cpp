#include "autoController.hpp"
#include "utils.hpp"

void RP::AutoController::start_auto()
{
    // timer.reset();
    init_turn();
}

void RP::AutoController::init_turn()
{
    printf("starting to turn...\n");
    turning = true;
    turnstate = SURVEY_COUNTERCW;
    orig_angle = agent.getRotation();
    tar_angle = orig_angle + AUTO_TURN_RANGE / 2.f;
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
            case SURVEY_COUNTERCW:
                if (closeEnough(agent.getInternalRotation(), tar_angle, 1e-4))
                {
                    turnstate = SURVEY_CW;
                    tar_angle = orig_angle - AUTO_TURN_RANGE / 2.f;
                    break;
                }
                printf("not there %f vs %f\n", agent.getInternalRotation(), tar_angle);
                grid.rotateAgent(agent, agent.turnTowards(tar_angle));
                break;
            case SURVEY_CW:
                if (closeEnough(agent.getInternalRotation(), tar_angle, 1e-4))
                {
                    turnstate = TOWARD_TARGET;
                    auto tar_point = map.compute_next_point();
                    tar_angle = atan2(tar_point.y - agent.getPosition().y, tar_point.x - agent.getPosition().x) * 180 / PI;
                    break;
                }
                grid.rotateAgent(agent, agent.turnTowards(tar_angle));
            case TOWARD_TARGET:
                if (closeEnough(agent.getInternalRotation(), tar_angle, 1e-4))
                {
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
        if (timer.elapsed() > AUTO_MOVE_TIME)
        {
            init_turn();
        }
        else 
        {
            grid.moveAgent(agent, agent.drive());
        }
    }
}

void RP::AutoController::stop_auto()
{
}