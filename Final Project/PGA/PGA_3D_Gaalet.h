#ifndef PGA_3D_GAALET_H
#define PGA_3D_GAALET_H

#include <cmath>
#include <cstdio>
#include <string>
#include <ostream>
#include <sstream>

// Gaalet + PGA3 headers
#include "cpp0x/gaalet.h"
#include "cpp0x/pga3.h"
#include "cpp0x/pga3_point.h"
#include "cpp0x/pga3_line.h"
#include "cpp0x/pga3_plane.h"
#include "cpp0x/pga3_ops.h"
#include "cpp0x/pga3_norm.h"

// 3D PGA using Gaalet

namespace pga_legacy
{
    using namespace gaalet;
    using namespace pga3;

    // Original multivector types
    using PointMV  = pga3::Point_t;
    using LineMV   = pga3::Line_t;
    using PlaneMV  = pga3::Plane_t;
    using MotorMV  = pga3::Motor_t;
    using Scalar   = pga3::space::algebra::element_t;

    // Small perturbation
    inline Scalar eps() { return Scalar(1e-7); }
    struct Pseudoscalar
    {
        PlaneMV::element_t s;
        explicit Pseudoscalar(Scalar v = Scalar(1.0)) : s(v) {}

        explicit Pseudoscalar(const pga3::space::mv<pga3::pseudoscalar_conf>::type& mv) {
            s = mv.template element<pga3::pseudoscalar_conf>();
        }

        pga3::space::mv<pga3::pseudoscalar_conf>::type mv() const{
            auto I = pga3::I;
            return s * I;
        }

        Scalar magnitude() const { 
            return std::fabs(s); 
        }

        Scalar magnitudeSqr() const { return s * s;
        }

        Pseudoscalar normalized() const { 
            return Pseudoscalar(Scalar(1)); 
        }

        operator std::string() const {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Pseudoscalar: %.5f", static_cast<double>(s));
            return std::string(buf);
        }

        void print(const char* title = "") const {
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    struct Dir3D {
        Scalar dx, dy, dz;

        Dir3D(Scalar x = 0, Scalar y = 0, Scalar z = 0) : dx(x), dy(y), dz(z) {}

        Scalar magnitude() const {
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        Scalar magnitudeSqr() const {
            return dx*dx + dy*dy + dz*dz;
        }

        Dir3D normalized() const {
            Scalar m = magnitude();
            if (m < eps()) return Dir3D(0,0,0);
            return Dir3D(dx/m, dy/m, dz/m);
        }

        Dir3D operator+(const Dir3D& rhs) const {
            return Dir3D(dx+rhs.dx, dy+rhs.dy, dz+rhs.dz);
        }

        Dir3D operator-(const Dir3D& rhs) const {
            return Dir3D(dx-rhs.dx, dy-rhs.dy, dz-rhs.dz);
        }

        Dir3D operator*(Scalar s) const {
            return Dir3D(dx*s, dy*s, dz*s);
        }

        friend Dir3D operator*(Scalar s, const Dir3D& d) {
            return d * s;
        }

        operator std::string() const{
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Dir3D: (%.5f, %.5f, %.5f)", static_cast<double>(dx), static_cast<double>(dy), static_cast<double>(dz));
            return std::string(buf);
        }

        void print(const char* title = "") const{
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    struct Point3D{
        PointMV mv;

        Point3D(Scalar x = 0, Scalar y = 0, Scalar z = 0) : mv(pga3::make_point(x, y, z)) {}

        explicit Point3D(const PointMV& mv_) : mv(mv_) {}

        Scalar x() const { return pga3::Point(mv).x(); }
        Scalar y() const { return pga3::Point(mv).y(); }
        Scalar z() const { return pga3::Point(mv).z(); }

        Point3D scaled(Scalar s) const {
            return Point3D(x()*s, y()*s, z()*s);
        }

        Point3D operator+(const Dir3D& d) const {
            return Point3D(x()+d.dx, y()+d.dy, z()+d.dz);
        }

        Point3D operator-(const Dir3D& d) const {
            return Point3D(x()-d.dx, y()-d.dy, z()-d.dz);
        }

        Dir3D operator-(const Point3D& rhs) const {
            return Dir3D(x()-rhs.x(), y()-rhs.y(), z()-rhs.z());
        }

        Point3D operator+(const Point3D& rhs) const {
            return Point3D(x()+rhs.x(), y()+rhs.y(), z()+rhs.z());
        }

        Scalar magnitude() const {
            return std::sqrt(x()*x() + y()*y() + z()*z());
        }

        Scalar magnitudeSqr() const {
            return x()*x() + y()*y() + z()*z();
        }

        Point3D normalized() const {
            Scalar m = magnitude();
            if (m < eps()) {
                return *this;
            }
            return Point3D(x()/m, y()/m, z()/m);
        }

        operator std::string() const {
            char buf[128];
            std::snprintf(buf, sizeof(buf),"Point3D: (%.5f, %.5f, %.5f)", static_cast<double>(x()), static_cast<double>(y()), static_cast<double>(z()));
            return std::string(buf);
        }

        void print(const char* title = "") const {
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    struct HomogeneousPoint3D {
        Scalar x_, y_, z_, w_;

        HomogeneousPoint3D(Scalar x = 0, Scalar y = 0,
                           Scalar z = 0, Scalar w = 1) : x_(x), y_(y), z_(z), w_(w) {}

        explicit HomogeneousPoint3D(const Point3D& p) : x_(p.x()), y_(p.y()), z_(p.z()), w_(1) {}

        Scalar x() const { return x_; }
        Scalar y() const { return y_; }
        Scalar z() const { return z_; }
        Scalar w() const { return w_; }

        Point3D toPoint() const {
            if (std::fabs(w_) < eps())
                return Point3D(x_, y_, z_);
            return Point3D(x_/w_, y_/w_, z_/w_);
        }

        operator std::string() const {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "HomogeneousPoint3D: (%.5f, %.5f, %.5f, %.5f)", static_cast<double>(x_), static_cast<double>(y_), static_cast<double>(z_), static_cast<double>(w_));
            return std::string(buf);
        }

        void print(const char* title = "") const {
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    struct Line3D
    {
        LineMV mv;

        Line3D(const Point3D& P, const Point3D& Q) : mv(pga3::line_from_points(P.mv, Q.mv)) {}

        Line3D(const Dir3D& d, const Point3D& p) : mv(pga3::make_line(p.x(), p.y(), p.z(), d.dx, d.dy, d.dz)) {}

        explicit Line3D(const LineMV& mv_) : mv(mv_) {}

        Dir3D direction() const {
            auto L = pga3::Line(mv);
            return Dir3D(L.i(), L.j(), L.k());
        }

        Dir3D moment() const {
            auto L = pga3::Line(mv);
            return Dir3D(L.dual_i(), L.dual_j(), L.dual_k());
        }

        Scalar magnitude() const {
            return pga3::norm(mv);
        }

        Scalar magnitudeSqr() const {
            Scalar m = pga3::norm(mv);
            return m*m;
        }

        Line3D normalized() const {
            return Line3D(pga3::normalize(mv));
        }

        operator std::string() const {
            auto d = direction();
            auto m = moment();
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Line3D: dir(%.5f, %.5f, %.5f), mom(%.5f, %.5f, %.5f)", static_cast<double>(d.dx), static_cast<double>(d.dy), static_cast<double>(d.dz), static_cast<double>(m.dx), static_cast<double>(m.dy), static_cast<double>(m.dz));
            return std::string(buf);
        }

        void print(const char* title = "") const {
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    struct IdealLine3D {
        Dir3D d;

        IdealLine3D(Scalar x = 0, Scalar y = 0, Scalar z = 0) : d(x,y,z) {}

        explicit IdealLine3D(const Dir3D& dir) : d(dir) {}

        Dir3D direction() const { return d; }

        Scalar magnitude() const { return d.magnitude(); }
        Scalar magnitudeSqr() const { return d.magnitudeSqr(); }

        IdealLine3D normalized() const {
            return IdealLine3D(d.normalized());
        }

        operator std::string() const {
            return std::string("IdealLine3D: ") + std::string(d);
        }

        void print(const char* title = "") const {
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    struct Plane3D {
        PlaneMV mv;

        Plane3D(Scalar a = 0, Scalar b = 0, Scalar c = 1, Scalar d = 0) : mv(pga3::make_plane(a,b,c,d)) {}

        explicit Plane3D(const PlaneMV& mv_) : mv(mv_) {}

        Scalar a() const { return pga3::Plane(mv).a(); }
        Scalar b() const { return pga3::Plane(mv).b(); }
        Scalar c() const { return pga3::Plane(mv).c(); }
        Scalar d() const { return pga3::Plane(mv).d(); }

        Dir3D normal() const {
            return Dir3D(a(), b(), c());
        }

        Scalar magnitude() const {
            return pga3::norm(mv);
        }

        Scalar magnitudeSqr() const {
            Scalar m = pga3::norm(mv);
            return m*m;
        }

        Plane3D normalized() const {
            return Plane3D(pga3::normalize(mv));
        }

        operator std::string() const {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Plane3D: %.5f X + %.5f Y + %.5f Z + %.5f = 0", static_cast<double>(a()), static_cast<double>(b()), static_cast<double>(c()), static_cast<double>(d()));
            return std::string(buf);
        }

        void print(const char* title = "") const {
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    struct Motor3D {
        MotorMV mv;

        Motor3D() : mv(pga3::one) {}

        explicit Motor3D(const MotorMV& mv_) : mv(mv_) {}

        static Motor3D Rotator(const Line3D& axis, Scalar angle) {
            return Motor3D(pga3::rotor(axis.mv, angle));
        }

        static Motor3D Translator(const Line3D& directionLine, Scalar distance) {
            return Motor3D(pga3::translator(directionLine.mv, distance));
        }

        static Motor3D Motor(const Line3D& line, Scalar distance, Scalar angle) {
            return Motor3D(pga3::motor(line.mv, distance, angle));
        }

        Motor3D operator*(const Motor3D& rhs) const {
            return Motor3D(eval(mv * rhs.mv));
        }

        Motor3D& operator*=(const Motor3D& rhs) {
            mv = eval(mv * rhs.mv);
            return *this;
        }

        Motor3D conjugate() const {
            return Motor3D(~mv);
        }

        Point3D operator()(const Point3D& p) const {
            auto out = eval(mv * p.mv * ~mv);
            return Point3D(out);
        }

        Dir3D operator()(const Dir3D& d) const {
            auto ideal = pga3::make_ideal_point(d.dx, d.dy, d.dz);
            auto out_mv = eval(mv * ideal * ~mv);
            auto P = pga3::Point(out_mv);
            return Dir3D(P.x(), P.y(), P.z());
        }

        Plane3D operator()(const Plane3D& pl) const {
            auto out = eval(mv * pl.mv * ~mv);
            return Plane3D(out);
        }

        Line3D operator()(const Line3D& L) const {
            auto out = eval(mv * L.mv * ~mv);
            return Line3D(out);
        }

        Scalar magnitude() const {
            return pga3::norm(mv);
        }

        Scalar magnitudeSqr() const {
            Scalar m = pga3::norm(mv);
            return m*m;
        }

        Motor3D normalized() const {
            Scalar m = magnitude();
            if (m < eps()) return *this;
            Scalar inv = Scalar(1) / m;
            return Motor3D(mv * inv);
        }

        operator std::string() const {
            std::ostringstream oss;
            oss << "Motor3D mv = " << eval(mv);
            return oss.str();
        }

        void print(const char* title = "") const {
            std::printf("%s%s\n", title, std::string(*this).c_str());
        }
    };

    inline Scalar dot(const Dir3D& a, const Dir3D& b) {
        return a.dx*b.dx + a.dy*b.dy + a.dz*b.dz;
    }

    inline Dir3D cross(const Dir3D& a, const Dir3D& b) {
        return Dir3D(
            a.dy*b.dz - a.dz*b.dy,
            a.dz*b.dx - a.dx*b.dz,
            a.dx*b.dy - a.dy*b.dx
        );
    }

    inline Scalar distance(const Point3D& a, const Point3D& b) {
        return (a-b).magnitude();
    }

    inline Point3D midpoint(const Point3D& a, const Point3D& b) {
        return Point3D(
            Scalar(0.5)*(a.x()+b.x()),
            Scalar(0.5)*(a.y()+b.y()),
            Scalar(0.5)*(a.z()+b.z())
        );
    }

    inline Line3D join(const Point3D& P, const Point3D& Q) {
        return Line3D(pga3::line_from_points(P.mv, Q.mv));
    }

    inline Plane3D join(const Point3D& P, const Point3D& Q, const Point3D& R) {
        auto mv = pga3::plane_from_points(P.mv, Q.mv, R.mv);
        return Plane3D(mv);
    }

    inline Point3D meet(const Line3D& L, const Plane3D& P) {
        auto mv = pga3::point_from_line_and_plane(L.mv, P.mv);
        return Point3D(mv);
    }

    inline Point3D meet(const Plane3D& P1, const Plane3D& P2, const Plane3D& P3) {
        auto mv = pga3::point_from_planes(P1.mv, P2.mv, P3.mv);
        return Point3D(mv);
    }

    inline Line3D meet(const Plane3D& P1, const Plane3D& P2) {
        auto mv = pga3::line_from_planes(P1.mv, P2.mv);
        return Line3D(mv);
    }

    inline std::ostream& operator<<(std::ostream& os, const Point3D& p) {
        return os << std::string(p);
    }

    inline std::ostream& operator<<(std::ostream& os, const Dir3D& d) {
        return os << std::string(d);
    }

    inline std::ostream& operator<<(std::ostream& os, const Line3D& L) {
        return os << std::string(L);
    }

    inline std::ostream& operator<<(std::ostream& os, const Plane3D& P) {
        return os << std::string(P);
    }

    inline std::ostream& operator<<(std::ostream& os, const Motor3D& M) {
        return os << std::string(M);
    }

}

// 4D PGA using the Gaalet library
namespace ga4d {

    typedef gaalet::algebra< gaalet::signature<4, 0, 0>, double > Algebra;

    typedef Algebra::mv<0x01, 0x02, 0x04, 0x08>::type Vec4;

    // Function to turn a 3D vector into 4D
    inline Vec4 make_vec4(pga_legacy::Scalar x, pga_legacy::Scalar y, pga_legacy::Scalar z, pga_legacy::Scalar w) {
        Vec4 v;
        v[0] = x;
        v[1] = y;
        v[2] = z;
        v[3] = w;
        return v;
    }

    inline pga_legacy::Scalar dot(const Vec4 &a, const Vec4 &b) {
        return  a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    inline Vec4 normalize(const Vec4 &v) {
        pga_legacy::Scalar len2 = dot(v, v);
        if (len2 <= pga_legacy::Scalar(0)) return v;
        pga_legacy::Scalar invLen = pga_legacy::Scalar(1) / std::sqrt(len2);
        return v * invLen;
    }

}

using pga_legacy::Scalar;
using pga_legacy::Pseudoscalar;
using pga_legacy::Dir3D;
using pga_legacy::Point3D;
using pga_legacy::HomogeneousPoint3D;
using pga_legacy::Line3D;
using pga_legacy::IdealLine3D;
using pga_legacy::Plane3D;
using pga_legacy::Motor3D;
using pga_legacy::dot;
using pga_legacy::cross;
using pga_legacy::distance;
using pga_legacy::midpoint;
using pga_legacy::join;
using pga_legacy::meet;

#endif
