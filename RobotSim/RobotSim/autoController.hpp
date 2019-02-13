#ifndef RP_AUTOCONTROLLLER
#define RP_AUTOCONTROLLLER

#include "grid.hpp"
#include "agent.hpp"
#include "timer.hpp"
#include "Map.hpp"

// for how long robot should move until it re-calculates and turns again
// TODO decrease this as robot gets closer
#define AUTO_MOVE_TIME 0.8f
// for how long robot should turn each time
#define AUTO_TURN_TIME 3.f

#define AUTO_TURN_RANGE 180

namespace RP
{
enum TurnState
{
    SURVEY_COUNTERCW,
    SURVEY_CW,
    TOWARD_TARGET
};
class AutoController
{
  public:
    AutoController(Grid &grd, Agent &agt, Map &mp) : grid(grd), agent(agt), map(mp){};
    void start_auto();
    void tic();
    void stop_auto();

  private:
    Grid &grid;
    Agent &agent;
    Map &map;
    Timer timer;
    float orig_angle; // angle before turning
    float tar_angle;  // target angle for turning
    bool turning;
    TurnState turnstate;

    void init_turn();
};
} // namespace RP

#endif