#include "utils.hpp"
#include <list>

namespace RP
{
class Mapper
{
  public:
    Mapper(const point &origin, const point& target, float bot_size);

    const point& cur;
    const point& tar;
    virtual std::vector<RP::point> compute_path()=0;
    void set_pos(const point& cur);
    void set_tar(const point& tar);
    void set_tol(float tol);
    void update_graph();
};
} // namespace RP
