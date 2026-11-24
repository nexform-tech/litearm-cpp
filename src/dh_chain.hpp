// Craig/MDH link frames for the LiteArm 7-DOF arm.
//
// This is the frame convention of the `DH/` folder, not the vendor URDF:
// every link frame is the controller's modified-DH (Craig) frame rebuilt in
// `DH/litearm_dh.urdf`.  Inertial parameters expressed in these frames can be
// compared directly with `DH/dh_params.yaml` and written to the firmware
// without a per-link frame conversion.
//
// The physical model is identical to the vendor URDF (same link axes, inertial
// origins and endpoint), but the coordinates of the same body differ by one
// constant frame transform per link.  Keep the exact DH rpy values
// (1.570796327 / 3.141592654 etc.): they are the `DH/litearm_dh.urdf` text,
// not a rounded rewrite.
//
// Joint signs: the controller's MDH frames put joints 4, 6 and 7 on -Z
// (`axis = s_i * z`), with s = {+1,+1,+1,-1,+1,-1,-1}.  The q convention is
// unchanged (`R(-z, q) = R(z, -q)`), so all recorded q / limits / direction
// values remain valid.
//
// `kDhCadBodies` is the CAD inertial data expressed in these DH link frames
// (the vendor URDF numbers rotated out of each inertial <origin> rpy); link 7
// already merges the fixed ee_link body (the firmware keeps body8 at zero
// mass).  Inertias are about the COM, expressed in the DH link frame.
#pragma once

#include <array>

#include "litearm/rot.hpp"

namespace litearm {
namespace ident_detail {

struct DhJoint {
    rot::Vec3 xyz;
    rot::Vec3 rpy;
    double sign;   ///< joint axis = sign * +Z in the child link frame
};

/// joint1..joint7 origins (parent -> child at q=0), from DH/litearm_dh.urdf.
inline constexpr std::array<DhJoint, 7> kDhJoints{{
    {{-2.236946042e-07, 7.534973224e-07, 0.1332499302}, {0.0, 0.0, 1.570792654}, 1.0},
    {{0.0, 0.0, 0.0}, {-1.570796327, 0.0, 0.0}, 1.0},
    {{0.0, -0.2995, 0.0}, {1.570796327, 0.0, 0.0}, 1.0},
    {{0.038, 0.0, 0.0}, {-1.570796327, 0.0, 0.0}, -1.0},
    {{-0.038, -0.2225, 0.0}, {1.570796327, 0.0, 0.0}, 1.0},
    {{0.0, 0.0, 0.0}, {0.0, 1.570796327, 1.570796327}, -1.0},
    {{-0.074, 0.0, 0.0}, {1.570796327, 0.0, 0.0}, -1.0},
}};

/// joint7 -> ee_link (fixed), from DH/litearm_dh.urdf.
inline constexpr DhJoint kDhEe{
    {-0.05374985299, 6.976277736e-07, 1.773739393e-06},
    {0.0, -1.570788097, -3.462916577e-06}, 0.0};

struct CadBody {
    double m;
    rot::Vec3 c;       ///< COM in the link frame (m)
    rot::Mat3 I_com;   ///< inertia about the COM, link frame (kg m^2)
};

/// CAD bodies (vendor URDF inertials) expressed in the MDH link frames.
/// Link 7 includes the fixed ee_link body (0.152315157261345 +
/// 0.0284157347604788 kg).
inline constexpr std::array<CadBody, 7> kDhCadBodies{{
    {0.636061100335262,
     {-0.0001447613913, 0.0001932544252, -0.02194391221},
     {{{0.00035685639071821067, 4.44212600824964e-08, -1.9576777316502063e-06},
       {4.44212600824964e-08, 0.0005549359023792373, 1.0199656140599972e-06},
       {-1.9576777316502063e-06, 1.0199656140599972e-06, 0.000409413887893391}}}},
    {0.296281105636482,
     {-0.0001100169416, -0.06504111819, 0.002006944166},
     {{{0.00027093671576553825, -2.3030200976792313e-08, 2.7093531908348074e-07},
       {-2.3030200976792309e-08, 0.00010040705839101081, 6.748766003963491e-08},
       {2.709353190834808e-07, 6.748766003963491e-08, 0.00028685231044397102}}}},
    {0.376902688786702,
     {0.0151168757, 1.276735608e-05, -0.05922060886},
     {{{0.00037666287922114272, -1.4378494363293508e-06, -2.8610804498991326e-05},
       {-1.4378494363293508e-06, 0.00043067245628824227, -1.3007159299116577e-06},
       {-2.8610804498991322e-05, -1.3007159299116575e-06, 0.00011104117173371801}}}},
    {0.285335817911565,
     {-0.03094796922, -0.04637086324, -0.001578315039},
     {{{0.00016389830028419057, -5.046845540758449e-05, -5.380337977873374e-07},
       {-5.046845540758449e-05, 0.00013561094652239566, -4.951345146201615e-07},
       {-5.380337977873374e-07, -4.951345146201615e-07, 0.00022597503614911281}}}},
    {0.352654558225702,
     {1.918213609e-05, -0.000426230303, -0.04194043113},
     {{{0.00017392986085147105, -3.2224887357035286e-08, -5.615158757019023e-08},
       {-3.2224887357035286e-08, 0.00020309553673201301, -3.310942685967663e-07},
       {-5.6151587570190237e-08, -3.310942685967663e-07, 7.255905130494363e-05}}}},
    {0.298759845757099,
     {-0.05252381675, -0.0002040631561, 0.001784336579},
     {{{0.0001732864823093237, 1.3971057846168223e-05, -1.3317618034367222e-05},
       {1.3971057846168223e-05, 0.00032755522325625722, 1.1753730483854915e-06},
       {-1.331761803436722e-05, 1.1753730483854915e-06, 0.00031378354071639303}}}},
    {0.1807308920218238,
     {-0.019235682744429699, -7.587784369761007e-05, -0.0016790265296914434},
     {{{5.791101622558416e-05, 9.402048294204919e-09, 2.046136313534214e-06},
       {9.402048294204906e-09, 8.356880591387089e-05, 3.0797050878756087e-07},
       {2.046136313534214e-06, 3.0797050878756087e-07, 0.00010683906752568633}}}},
}};

}  // namespace ident_detail
}  // namespace litearm
