// vqdot.vx vd, vs2, rs1, vm
#include "vqdot_common.h"

require_extension(EXT_ZVQDOTQ);
require(P.VU.vsew == e32);

VI_VX_LOOP
({
  VQDOT(rs1, vs2, int8_t, int8_t);
  vd = (vd + result) & 0xffffffff;
})
