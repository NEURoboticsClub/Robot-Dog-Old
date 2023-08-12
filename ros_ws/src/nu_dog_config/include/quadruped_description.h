#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin(0.2, 0.19035, 0.52, 0.0, 0.0, -1.5708);
base.lf.upper_leg.setOrigin(-0.087, -0.04635, -0.0, 0.0, 0.0, 0.0);
base.lf.lower_leg.setOrigin(0.002279, 0.58172, -0.3055, 0.0, 0.0, 0.0);
     base.lf.foot.setOrigin(-0.0018790135315, -0.6416355, -0.306958515898, 0.0, 0.0, 0.0);

      base.rf.hip.setOrigin(0.2, -0.19035, 0.52, 0.0, 0.0, -1.5708);
base.rf.upper_leg.setOrigin(-0.087, 0.04635, -0.0, 0.0, 0.0, 0.0);
base.rf.lower_leg.setOrigin(0.002279, 0.089024, -0.3055, 0.0, 0.0, 0.0);
     base.rf.foot.setOrigin(-0.002679, -0.029112, -0.30696, 0.0, 0.0, 0.0);

      base.lh.hip.setOrigin(-0.2, 0.72405, 0.52, 0.0, 0.0, -1.5708);
base.lh.upper_leg.setOrigin(0.087, -0.04635, -0.0, 0.0, 0.0, 0.0);
base.lh.lower_leg.setOrigin(-0.0025586, -0.089024, -0.30666, 0.0, 0.0, 0.0);
     base.lh.foot.setOrigin(0.00295863837266, 0.0291125, -0.305796245574, 0.0, 0.0, 0.0);

      base.rh.hip.setOrigin(-0.2, -0.72405, 0.52, 0.0, 0.0, -1.5708);
base.rh.upper_leg.setOrigin(0.087, 0.04635, -0.0, 0.0, 0.0, 0.0);
base.rh.lower_leg.setOrigin(0.0022494, 0.088715, -0.30665, 0.0, 0.0, 0.0);
     base.rh.foot.setOrigin(-0.0026494487696, -0.0288031034448, -0.305808448116, 0.0, 0.0, 0.0);
        }
    }
}
#endif