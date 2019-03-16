#include "utils.hpp"
#include <list>

namespace RP
{
class Mapper
{
  public:
    Mapper(const point &origin, const point& target, float bot_size);

    virtual void update(const std::list<line>& new_obstacles);
};
} // namespace RP
