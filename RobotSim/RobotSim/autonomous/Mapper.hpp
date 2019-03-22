#ifndef RP_MAPPER_HPP
#define RP_MAPPER_HPP

#include "utils.hpp"
#include <iterator>

namespace RP
{
class Mapper
{
  public:
    Mapper(const point &origin, const point& target, float bot_size);

    const point& cur;
    const point& tar;
    void set_pos(const point& cur);
    void set_tar(const point& tar);
    void set_tol(float tol);
    virtual void update_graph()=0;
    // graph methods used for pather
    virtual const std::vector<int>& neighbors(int node)=0;
};
} // namespace RP

#endif
