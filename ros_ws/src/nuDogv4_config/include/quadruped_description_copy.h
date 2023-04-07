#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin(0.2, 0.19035, 0.52, 1.5708, -1.5708, 0.0);
base.lf.upper_leg.setOrigin(0.0, 0.04635, -0.087, -1.5708, 0.78534, 3.1416);
base.lf.lower_leg.setOrigin(-0.00025, -0.3066, -0.57642, 0.0, 0.0, 0.37477);
     base.lf.foot.setOrigin(0.0004653, -0.00018303, -0.63653, 3.1416, 0.0, -2.1201);

      base.rf.hip.setOrigin(0.2, -0.19035, 0.52, 1.5708, -1.5708, 0.0);
base.rf.upper_leg.setOrigin(0.0, -0.04635, -0.087, -1.5708, -0.78534, 0.0);
base.rf.lower_leg.setOrigin(0.00025, -0.3066, -0.083724, 3.1416, 0.0, 2.7668);
     base.rf.foot.setOrigin(0.0, 0.0, -0.0, 0.0, 0.0, 0.0);

      base.lh.hip.setOrigin(-0.2, 0.72405, 0.52, 1.5708, 1.5708, 0.0);
base.lh.upper_leg.setOrigin(0.0, -0.04635, -0.087, -1.5708, 0.78534, 0.0);
base.lh.lower_leg.setOrigin(0.00025, -0.3066, -0.060105, 3.1416, 0.0, -1.7453);
     base.lh.foot.setOrigin(0.0, 0.0, -0.0, 0.0, 0.0, 0.0);

      base.rh.hip.setOrigin(-0.2, -0.72405, 0.52, 1.5708, 1.5708, 0.0);
base.rh.upper_leg.setOrigin(0.0, 0.04635, -0.087, -1.5708, -0.78534, -3.1416);
base.rh.lower_leg.setOrigin(0.00025, -0.3066, -0.077374, 0.0, 0.0, -0.023244);
     base.rh.foot.setOrigin(0.0, 0.0, -0.0, 0.0, 0.0, 0.0);
        }
    }
}
#endif