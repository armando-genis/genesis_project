#ifndef PATHSETTINGS_H
#define PATHSETTINGS_H

#include <vector>
#include "RSCar.h"
#include "PathSegmentLengths.h"
#include "PathWords.h"
#include <cmath>

namespace PathfindingForVehicles {
    namespace ReedsSheppPaths {
        class PathSettings {
        public:
            struct SegmentSettings {
                RSCar::Steering steering;
                RSCar::Gear gear;
                float length;

                SegmentSettings(RSCar::Steering steering, RSCar::Gear gear, float length)
                    : steering(steering), gear(gear), length(length) {}
            };

            static std::vector<SegmentSettings> GetSettings(PathWords word, const PathSegmentLengths& pathLengths) {
                switch (word) {
                    //8.1: CSC, same turn
                    case PathWords::Lf_Sf_Lf: return Lf_Sf_Lf_path(pathLengths);
                    case PathWords::Lb_Sb_Lb: return TimeFlip(Lf_Sf_Lf_path(pathLengths));
                    case PathWords::Rf_Sf_Rf: return Reflect(Lf_Sf_Lf_path(pathLengths));
                    case PathWords::Rb_Sb_Rb: return Reflect(TimeFlip(Lf_Sf_Lf_path(pathLengths)));

                    //8.2: CSC, different turn
                    case PathWords::Lf_Sf_Rf: return Lf_Sf_Rf_path(pathLengths);
                    case PathWords::Lb_Sb_Rb: return TimeFlip(Lf_Sf_Rf_path(pathLengths));
                    case PathWords::Rf_Sf_Lf: return Reflect(Lf_Sf_Rf_path(pathLengths));
                    case PathWords::Rb_Sb_Lb: return Reflect(TimeFlip(Lf_Sf_Rf_path(pathLengths)));

                    //8.3: C|C|C
                    case PathWords::Lf_Rb_Lf: return Lf_Rb_Lf_path(pathLengths);
                    case PathWords::Lb_Rf_Lb: return TimeFlip(Lf_Rb_Lf_path(pathLengths));
                    case PathWords::Rf_Lb_Rf: return Reflect(Lf_Rb_Lf_path(pathLengths));
                    case PathWords::Rb_Lf_Rb: return Reflect(TimeFlip(Lf_Rb_Lf_path(pathLengths)));

                    //8.4: C|CC
                    case PathWords::Lf_Rb_Lb: return Lf_Rb_Lb_path(pathLengths);
                    case PathWords::Lb_Rf_Lf: return TimeFlip(Lf_Rb_Lb_path(pathLengths));
                    case PathWords::Rf_Lb_Rb: return Reflect(Lf_Rb_Lb_path(pathLengths));
                    case PathWords::Rb_Lf_Rf: return Reflect(TimeFlip(Lf_Rb_Lb_path(pathLengths)));

                    //8.4: CC|C
                    case PathWords::Lf_Rf_Lb: return Lf_Rf_Lb_path(pathLengths);
                    case PathWords::Lb_Rb_Lf: return TimeFlip(Lf_Rf_Lb_path(pathLengths));
                    case PathWords::Rf_Lf_Rb: return Reflect(Lf_Rf_Lb_path(pathLengths));
                    case PathWords::Rb_Lb_Rf: return Reflect(TimeFlip(Lf_Rf_Lb_path(pathLengths)));

                    //8.7: CCu|CuC
                    case PathWords::Lf_Ruf_Lub_Rb: return Lf_Ruf_Lub_Rb_path(pathLengths);
                    case PathWords::Lb_Rub_Luf_Rf: return TimeFlip(Lf_Ruf_Lub_Rb_path(pathLengths));
                    case PathWords::Rf_Luf_Rub_Lb: return Reflect(Lf_Ruf_Lub_Rb_path(pathLengths));
                    case PathWords::Rb_Lub_Ruf_Lf: return Reflect(TimeFlip(Lf_Ruf_Lub_Rb_path(pathLengths)));

                    //8.8: C|CuCu|C
                    case PathWords::Lf_Rub_Lub_Rf: return Lf_Rub_Lub_Rf_path(pathLengths);
                    case PathWords::Lb_Ruf_Luf_Rb: return TimeFlip(Lf_Rub_Lub_Rf_path(pathLengths));
                    case PathWords::Rf_Lub_Rub_Lf: return Reflect(Lf_Rub_Lub_Rf_path(pathLengths));
                    case PathWords::Rb_Luf_Ruf_Lb: return Reflect(TimeFlip(Lf_Rub_Lub_Rf_path(pathLengths)));

                    //8.9: C|C(pi/2)SC, same turn
                    case PathWords::Lf_Rbpi2_Sb_Lb: return Lf_Rbpi2_Sb_Lb_path(pathLengths);
                    case PathWords::Lb_Rfpi2_Sf_Lf: return TimeFlip(Lf_Rbpi2_Sb_Lb_path(pathLengths));
                    case PathWords::Rf_Lbpi2_Sb_Rb: return Reflect(Lf_Rbpi2_Sb_Lb_path(pathLengths));
                    case PathWords::Rb_Lfpi2_Sf_Rf: return Reflect(TimeFlip(Lf_Rbpi2_Sb_Lb_path(pathLengths)));

                    //8.10: C|C(pi/2)SC, different turn
                    case PathWords::Lf_Rbpi2_Sb_Rb: return Lf_Rbpi2_Sb_Rb_path(pathLengths);
                    case PathWords::Lb_Rfpi2_Sf_Rf: return TimeFlip(Lf_Rbpi2_Sb_Rb_path(pathLengths));
                    case PathWords::Rf_Lbpi2_Sb_Lb: return Reflect(Lf_Rbpi2_Sb_Rb_path(pathLengths));
                    case PathWords::Rb_Lfpi2_Sf_Lf: return Reflect(TimeFlip(Lf_Rbpi2_Sb_Rb_path(pathLengths)));

                    //8.9 (reversed): CSC(pi/2)|C, same turn
                    case PathWords::Lf_Sf_Rfpi2_Lb: return Lf_Sf_Rfpi2_Lb_path(pathLengths);
                    case PathWords::Lb_Sb_Rbpi2_Lf: return TimeFlip(Lf_Sf_Rfpi2_Lb_path(pathLengths));
                    case PathWords::Rf_Sf_Lfpi2_Rb: return Reflect(Lf_Sf_Rfpi2_Lb_path(pathLengths));
                    case PathWords::Rb_Sb_Lbpi2_Rf: return Reflect(TimeFlip(Lf_Sf_Rfpi2_Lb_path(pathLengths)));

                    //8.10 (reversed): CSC(pi/2)|C, different turn
                    case PathWords::Lf_Sf_Lfpi2_Rb: return Lf_Sf_Lfpi2_Rb_path(pathLengths);
                    case PathWords::Lb_Sb_Lbpi2_Rf: return TimeFlip(Lf_Sf_Lfpi2_Rb_path(pathLengths));
                    case PathWords::Rf_Sf_Rfpi2_Lb: return Reflect(Lf_Sf_Lfpi2_Rb_path(pathLengths));
                    case PathWords::Rb_Sb_Rbpi2_Lf: return Reflect(TimeFlip(Lf_Sf_Lfpi2_Rb_path(pathLengths)));

                    //8.11: C|C(pi/2)SC(pi/2)|C
                    case PathWords::Lf_Rbpi2_Sb_Lbpi2_Rf: return Lf_Rbpi2_Sb_Lbpi2_Rf_path(pathLengths);
                    case PathWords::Lb_Rfpi2_Sf_Lfpi2_Rb: return TimeFlip(Lf_Rbpi2_Sb_Lbpi2_Rf_path(pathLengths));
                    case PathWords::Rf_Lbpi2_Sb_Rbpi2_Lf: return Reflect(Lf_Rbpi2_Sb_Lbpi2_Rf_path(pathLengths));
                    case PathWords::Rb_Lfpi2_Sf_Rfpi2_Lb: return Reflect(TimeFlip(Lf_Rbpi2_Sb_Lbpi2_Rf_path(pathLengths)));
                }
                return {};
            }

        private:
            //Time-flip transform method from the report, which interchanges + and -
            static std::vector<SegmentSettings> TimeFlip(std::vector<SegmentSettings> pathSettings) {
                for (auto& settings : pathSettings) {
                    settings.gear = (settings.gear == RSCar::Gear::Forward) ? RSCar::Gear::Back : RSCar::Gear::Forward;
                }
                return pathSettings;
            }

            //Reflect transform from the report, which interchanges r and l 
            static std::vector<SegmentSettings> Reflect(std::vector<SegmentSettings> pathSettings) {
                for (auto& settings : pathSettings) {
                    if (settings.steering == RSCar::Steering::Straight) continue;
                    settings.steering = (settings.steering == RSCar::Steering::Right) ? RSCar::Steering::Left : RSCar::Steering::Right;
                }
                return pathSettings;
            }

            // Settings for individual paths
            //8.1: CSC, same turn
            static std::vector<SegmentSettings> Lf_Sf_Lf_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.v)
                };
            }

            //8.2: CSC, different turn
            static std::vector<SegmentSettings> Lf_Sf_Rf_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.v)
                };
            }

            //8.3: C|C|C
            static std::vector<SegmentSettings> Lf_Rb_Lf_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.v)
                };
            }

            //8.4: C|CC
            static std::vector<SegmentSettings> Lf_Rb_Lb_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v)
                };
            }

            //8.4: CC|C
            static std::vector<SegmentSettings> Lf_Rf_Lb_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v)
                };
            }

            //8.7: CCu|CuC
            static std::vector<SegmentSettings> Lf_Ruf_Lub_Rb_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.v)
                };
            }

            //8.8: C|CuCu|C
            static std::vector<SegmentSettings> Lf_Rub_Lub_Rf_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.v)
                };
            }

            //8.9: C|C(pi/2)SC, same turn
            static std::vector<SegmentSettings> Lf_Rbpi2_Sb_Lb_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, M_PI / 2),
                    SegmentSettings(RSCar::Steering::Straight, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v)
                };
            }

            //8.10: C|C(pi/2)SC, different turn
            static std::vector<SegmentSettings> Lf_Rbpi2_Sb_Rb_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, M_PI / 2),
                    SegmentSettings(RSCar::Steering::Straight, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.v)
                };
            }

            //8.9 (reversed): CSC(pi/2)|C, same turn
            static std::vector<SegmentSettings> Lf_Sf_Rfpi2_Lb_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Forward, M_PI / 2),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v)
                };
            }

            //8.10 (reversed): CSC(pi/2)|C, different turn
            static std::vector<SegmentSettings> Lf_Sf_Lfpi2_Rb_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, M_PI / 2),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.v)
                };
            }

            //8.11: C|C(pi/2)SC(pi/2)|C
            static std::vector<SegmentSettings> Lf_Rbpi2_Sb_Lbpi2_Rf_path(const PathSegmentLengths& pathLength) {
                return {
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Back, M_PI / 2),
                    SegmentSettings(RSCar::Steering::Straight, RSCar::Gear::Back, pathLength.u),
                    SegmentSettings(RSCar::Steering::Left, RSCar::Gear::Back, M_PI / 2),
                    SegmentSettings(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.v)
                };
            }
        };
    }
}

#endif // PATHSETTINGS_H
