#include "utils.hpp"
#include <map>

namespace RP
{
struct range {
    
}

class GPSSim
{
    public:
        GPSSim();
        RP::point generate_pt(float err, float true_lat, float true_lng);
    private:
        
        float convert_to_gps_err(float err_meters);
};
}

