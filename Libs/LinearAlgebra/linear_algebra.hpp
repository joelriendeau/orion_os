// WORK IN PROGRESS
// This should replace lin_algebra.hpp eventually
// Objectives : define all operators using either static or dynamic operands and result types (8 versions for each operations as non-member namespaced functions)
//              define special matrix types for diagonal matrices to speed up operations such as mul, inv, etc. while saving memory
//              define sub-matrices out of target matrix, extract and set vectors into matrices (sub-matrix accessors)
//              publishable quality

#pragma once

#include "Types/types.hpp"
#include "Types/assert.hpp"
#include "boost/static_assert.hpp"
#include "cmath"

#if defined(min) || defined(max) // eigen does not want min or max defined
    #undef min
    #undef max
#endif

namespace lin_alg
{
    static const double quarter_pi = 0.785398163397448309616;
    static const double half_pi = 1.57079632679489661923;
    static const double pi = 3.14159265358979323846;
    static const double two_pi = pi * 2;

    template<typename T> inline T clamp(const T& a, const T& mi, const T& ma) {return max_t(min_t(a, ma), mi);}
    template<typename T> inline T deg_to_rad(const T& deg) {return deg * static_cast<T>(pi / 180.);}
    template<typename T> inline T rad_to_deg(const T& rad) {return rad * static_cast<T>(180. / pi);}

    namespace structural_ids
    {
        enum en
        {
            normal = 0,
            diagonal,
        };
    }

    template <typename T, structural_ids::en S> class dynamic_buffer;

    // static_buffer declares its memory on the stack
    template <typename T, u8 R, u8 C, structural_ids::en S = structural_ids::normal>
    class static_buffer
    {
    private:
        BOOST_STATIC_ASSERT(R > 0);
        BOOST_STATIC_ASSERT(C > 0);
    public:
        typedef T T;
        static const u8 R = R;
        static const u8 C = C;
        inline u8 get_R() const { return R; }
        inline u8 get_C() const { return C; }
        inline static_buffer() {}
        template <structural_ids::en S2>
        inline static_buffer(const static_buffer<T,R,C,S2>& from)
        {
            for (u32 i = 0; i < R*C; ++i) m[i] = from.get(i);
        }
        inline static_buffer(const static_buffer<T,R,C,S>& from)
        {
            for (u32 i = 0; i < R*C; ++i) m[i] = from.get(i);
        }
        template <structural_ids::en S2>
        inline static_buffer(const dynamic_buffer<T, S2>& from)
        {
            assert(R == from.get_R() && C == from.get_C());
            for (u32 i = 0; i < R*C; ++i) m[i] = from.get(i);
        }
        inline static_buffer(T from)
        {
            for (u32 i = 0; i < R*C; ++i) m[i] = from;
        }
        template <structural_ids::en S2>
        inline static_buffer<T,R,C,S>& operator=(const static_buffer<T,R,C,S2>& from)
        {
            for (u32 i = 0; i < R*C; ++i) m[i] = from.get(i);
            return *this;
        }
        inline static_buffer<T,R,C,S>& operator=(const static_buffer<T,R,C,S>& from)
        {
            for (u32 i = 0; i < R*C; ++i) m[i] = from.get(i);
            return *this;
        }
        template <structural_ids::en S2>
        inline static_buffer<T,R,C,S>& operator=(const dynamic_buffer<T,S2>& from)
        {
            assert(R == from.get_R() && C == from.get_C());
            for (u32 i = 0; i < R*C; ++i) m[i] = from.get(i);
            return *this;
        }
        inline static_buffer<T,R,C,S>& operator=(T from)
        {
            for (u32 i = 0; i < R*C; ++i) m[i] = from;
            return *this;
        }
        inline const T get(u32 index) const { assert(index < static_cast<u32>(R*C)); return m[index]; }
        inline const T get(u8 r, u8 c) const { assert(r < R && c < C); return m[R*c + r]; }
        inline T& set(u32 index) { assert(index < static_cast<u32>(R*C)); return m[index]; }
        inline T& set(u8 r, u8 c) { assert(r < R && c < C); return m[R*c + r]; }
        T m[R*C];
    };

    template <typename T, u8 R, u8 C>
    class static_buffer<T, R, C, structural_ids::diagonal>
    {
    private:
        BOOST_STATIC_ASSERT(R > 0);
        BOOST_STATIC_ASSERT(C > 0);
        BOOST_STATIC_ASSERT(R == C);
    public:
        typedef T T;
        static const u8 R = R;
        static const u8 C = C;
        inline u8 get_R() const { return R; }
        inline u8 get_C() const { return C; }
        inline static_buffer() {}
        template <structural_ids::en S2>
        inline static_buffer(const static_buffer<T,R,C,S2>& from)
        {
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < C; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
        }
        inline static_buffer(const static_buffer<T,R,C,structural_ids::diagonal>& from)
        {
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < C; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
        }
        template <structural_ids::en S2>
        inline static_buffer(const dynamic_buffer<T,S2>& from)
        {
            assert(R == from.get_R() && R == from.get_C());
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < C; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
        }
        inline static_buffer(T from)
        {
            for (u8 i = 0; i < R; ++i) m[i] = from;
        }
        template <structural_ids::en S2>
        inline static_buffer<T,R,C,structural_ids::diagonal>& operator=(const static_buffer<T,R,C,S2>& from)
        {
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < C; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
            return *this;
        }
        inline static_buffer<T,R,C,structural_ids::diagonal>& operator=(const static_buffer<T,R,C,structural_ids::diagonal>& from)
        {
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < C; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
            return *this;
        }
        template <structural_ids::en S2>
        inline static_buffer<T,R,C,structural_ids::diagonal>& operator=(const dynamic_buffer<T,S2>& from)
        {
            assert(R == from.get_R() && R == from.get_C());
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < C; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
            return *this;
        }
        inline static_buffer<T,R,C,structural_ids::diagonal>& operator=(T from)
        {
            for (u8 i = 0; i < R; ++i) m[i] = from;
            return *this;
        }
        inline const T get(u32 index) const { assert(index < static_cast<u32>(R*C)); u32 r = index / C; u32 c = index % C; return (r == c) ? m[r] : static_cast<T>(0); }
        inline const T get(u8 r, u8 c) const { assert(r < R && c < C); return (r == c) ? m[r] : static_cast<T>(0); }
        inline T& set(u32 index) { assert(index < static_cast<u32>(R*C)); u32 r = index / C; u32 c = index % C; assert(r==c); return m[r]; }
        inline T& set(u8 r, u8 c) { assert(r < R && r == c); return m[r]; }
        T m[R];
    };

    // dynamic_buffer allocates its memory on the heap.
    template <typename T, structural_ids::en S = structural_ids::normal>
    class dynamic_buffer
    {
    private:
        u8 R;
        u8 C;
    public:
        typedef T T;
        inline u8 get_R() const { return R; }
        inline u8 get_C() const { return C; }
        inline ~dynamic_buffer()
        {
            delete [] m;
        }
        inline dynamic_buffer(u8 rows, u8 columns)
        {
            assert(rows > 0);
            assert(columns > 0);
            R = rows;
            C = columns;
            m = new T[R*C];
        }
        inline dynamic_buffer(u8 rows, u8 columns, T from)
        {
            assert(rows > 0);
            assert(columns > 0);
            R = rows;
            C = columns;
            m = new T[R*C];
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from;
        }
        template <u8 R2, u8 C2, structural_ids::en S2>
        inline dynamic_buffer(const static_buffer<T,R2,C2,S2>& from)
        {
            R = R2;
            C = C2;
            m = new T[R*C];
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from.get(i);
        }
        template <structural_ids::en S2>
        inline dynamic_buffer(const dynamic_buffer<T,S2>& from)
        {
            R = from.get_R();
            C = from.get_C();
            m = new T[R*C];
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from.get(i);
        }
        inline dynamic_buffer(const dynamic_buffer<T,S>& from)
        {
            R = from.get_R();
            C = from.get_C();
            m = new T[R*C];
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from.get(i);
        }
        template <u8 R2, u8 C2, structural_ids::en S2>
        inline dynamic_buffer<T,S>& operator=(const static_buffer<T,R2,C2,S2>& from)
        {
            assert(R == R2 && C == C2);
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from.get(i);
            return *this;
        }
        template <structural_ids::en S2>
        inline dynamic_buffer<T,S>& operator=(const dynamic_buffer<T,S2>& from)
        {
            assert(R == from.get_R() && C == from.get_C());
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from.get(i);
            return *this;
        }
        inline dynamic_buffer<T,S>& operator=(const dynamic_buffer<T,S>& from)
        {
            assert(R == from.get_R() && C == from.get_C());
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from.get(i);
            return *this;
        }
        inline dynamic_buffer<T,S>& operator=(T from)
        {
            for (u32 i = 0; i < static_cast<u32>(R*C); ++i) m[i] = from;
            return *this;
        }
        inline const T get(u32 index) const { assert(index < static_cast<u32>(R*C)); return m[index]; }
        inline const T get(u8 r, u8 c) const { assert(r < R && c < C); return m[R*c + r]; }
        inline T& set(u32 index) { assert(index < static_cast<u32>(R*C)); return m[index]; }
        inline T& set(u8 r, u8 c) { assert(r < R && c < C); return m[R*c + r]; }
        T* m;
    };

    template <typename T>
    class dynamic_buffer<T, structural_ids::diagonal>
    {
    private:
        u8 R;
    public:
        typedef T T;
        inline u8 get_R() const { return R; }
        inline u8 get_C() const { return R; }
        inline ~dynamic_buffer()
        {
            delete [] m;
        }
        inline dynamic_buffer(u8 rows, u8 columns)
        {
            assert(rows > 0);
            assert(rows == columns);
            R = rows;
            m = new T[R];
        }
        inline dynamic_buffer(u8 rows, u8 columns, T from)
        {
            assert(rows > 0);
            assert(rows == columns);
            R = rows;
            m = new T[R];
            for (u8 i = 0; i < R; ++i) m[i] = from;
        }
        template <u8 R2, structural_ids::en S2>
        inline dynamic_buffer(const static_buffer<T,R2,R2,S2>& from)
        {
            R = R2;
            m = new T[R];
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < R; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
        }
        template <structural_ids::en S2>
        inline dynamic_buffer(const dynamic_buffer<T,S2>& from)
        {
            assert(from.get_R() == from.get_C());
            R = from.get_R();
            m = new T[R];
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < R; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
        }
        inline dynamic_buffer(const dynamic_buffer<T,structural_ids::diagonal>& from)
        {
            assert(from.get_R() == from.get_C());
            R = from.get_R();
            m = new T[R];
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < R; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
        }
        template <u8 R2, structural_ids::en S2>
        inline dynamic_buffer<T,structural_ids::diagonal>& operator=(const static_buffer<T,R2,R2,S2>& from)
        {
            assert(R == R2);
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < R; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
            return *this;
        }
        template <structural_ids::en S2>
        inline dynamic_buffer<T,structural_ids::diagonal>& operator=(const dynamic_buffer<T,S2>& from)
        {
            assert(R == from.get_R() && R == from.get_C());
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < R; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
            return *this;
        }
        inline dynamic_buffer<T,structural_ids::diagonal>& operator=(const dynamic_buffer<T,structural_ids::diagonal>& from)
        {
            assert(R == from.get_R() && R == from.get_C());
            for (u8 i = 0; i < R; ++i)
            {
                for (u8 j = 0; j < R; ++j)
                {
                    if (i == j)
                        m[i] = from.get(i, j);
                    else
                    {
                        assert(from.get(i, j) == static_cast<T>(0));
                    }
                }
            }
            return *this;
        }
        inline dynamic_buffer<T,structural_ids::diagonal>& operator=(T from)
        {
            for (u8 i = 0; i < R; ++i) m[i] = from;
            return *this;
        }
        inline const T get(u32 index) const { assert(index < static_cast<u32>(R*R)); u32 r = index / R; u32 c = index % R; return (r == c) ? m[r] : static_cast<T>(0); }
        inline const T get(u8 r, u8 c) const { assert(r < R && c < C); return (r == c) ? m[r] : static_cast<T>(0); }
        inline T& set(u32 index) { assert(index < static_cast<u32>(R*C)); u32 r = index / R; u32 c = index % R; assert(r==c); return m[r]; }
        inline T& set(u8 r, u8 c) { assert(r < R && r == c); return m[r]; }
        T* m;
    };

    template <typename T, structural_ids::en S = structural_ids::normal> class dynamic_matrix;

    // static_matrix declares its memory on the stack. use this version when you know the dimensions in advance.
    template <typename T, u8 R, u8 C, structural_ids::en S = structural_ids::normal>
    class static_matrix
    {
    public:
        typedef T T;
        static_buffer<T,R,C,S> buf;

        inline const typename T get(u32 index) const { return buf.get(index); }
        inline const typename T get(u8 r, u8 c) const { return buf.get(r, c); }
        inline typename T& set(u32 index) { return buf.set(index); }
        inline typename T& set(u8 r, u8 c) { return buf.set(r, c); }

        static_matrix() {}
        template <structural_ids::en S2>
        static_matrix(const static_matrix<T,R,C,S2>& from) : buf(from.buf) { }
        static_matrix(const static_matrix<T,R,C,S>& from) : buf(from.buf) { }
        template <structural_ids::en S2>
        static_matrix(const dynamic_matrix<T,S2>& from) : buf(from.buf) { }
        static_matrix(T from) : buf(from) { }

        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S>& operator=(const static_matrix<T,R,C,S2>& from) { buf = from.buf; return *this; }
        inline static_matrix<T,R,C,S>& operator=(const static_matrix<T,R,C,S>& from) { buf = from.buf; return *this; }
        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S>& operator=(const dynamic_matrix<T,S2>& from) { buf = from.buf; return *this; }
        inline static_matrix<T,R,C,S>& operator=(T from) { buf = from; return *this; }

        inline void init(T v0)
        {
            BOOST_STATIC_ASSERT(R > 0 && C == 1);
            set(0) = v0;
            for (u8 i = 1; i < R; i++) set(i) = 0;
        }
        inline void init(T v0, T v1)
        {
            BOOST_STATIC_ASSERT(R > 1 && C == 1);
            set(0) = v0;
            set(1) = v1;
            for (u8 i = 2; i < R; i++) set(i) = 0;
        }
        inline void init(T v0, T v1, T v2)
        {
            BOOST_STATIC_ASSERT(R > 2 && C == 1);
            set(0) = v0;
            set(1) = v1;
            set(2) = v2;
            for (u8 i = 3; i < R; i++) set(i) = 0;
        }
        inline void init(T v0, T v1, T v2, T v3)
        {
            BOOST_STATIC_ASSERT((R > 3) && (C == 1));
            set(0) = v0;
            set(1) = v1;
            set(2) = v2;
            set(3) = v3;
            for (u8 i = 4; i < R; i++) set(i) = 0;
        }

        inline void identity()
        {
            BOOST_STATIC_ASSERT(R == C);
            private_ops::identity_t<R,S>::identity(*this);
        }

        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S> operator+(const static_matrix<T,R,C,S2>& a) const
        {
            static_matrix<T,R,C,S> result;
            private_ops::add_t<R,C>(*this, a, result);
            return result;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S> operator+(const dynamic_matrix<T,S2>& a) const
        {
            assert(R == a.get_R() && C == a.get_C());
            static_matrix<T,R,C,S> result;
            private_ops::add_t<R,C>(*this, a, result);
            return result;
        }
        inline static_matrix<T,R,C,S> operator+(T a) const
        {
            static_matrix<T,R,C,S> result;
            private_ops::add_t<R,C>(*this, a, result);
            return result;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S>& operator+=(const static_matrix<T,R,C,S2>& a)
        {
            private_ops::add_t<R,C>(*this, a, *this);
            return *this;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S>& operator+=(const dynamic_matrix<T,S2>& a)
        {
            assert(R == a.get_R() && C == a.get_C());
            private_ops::add_t<R,C>(*this, a, *this);
            return *this;
        }
        inline static_matrix<T,R,C,S>& operator+=(T a)
        {
            private_ops::add_t<R, C>(*this, a, *this);
            return *this;
        }

        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S> operator-(const static_matrix<T,R,C,S2>& a) const
        {
            static_matrix<T,R,C> result;
            private_ops::sub_t<R,C>(*this, a, result);
            return result;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S> operator-(const dynamic_matrix<T,S2>& a) const
        {
            assert(R == a.get_R() && C == a.get_C());
            static_matrix<T,R,C> result;
            private_ops::sub_t<R,C>(*this, a, result);
            return result;
        }
        inline static_matrix<T,R,C,S> operator-(T a) const
        {
            static_matrix<T,R,C> result;
            private_ops::sub_t<R, C>(*this, a, result);
            return result;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S>& operator-=(const static_matrix<T,R,C,S2>& a)
        {
            private_ops::sub_t<R,C>(*this, a, *this);
            return *this;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,C,S>& operator-=(const dynamic_matrix<T,S2>& a)
        {
            assert(R == a.get_R() && C == a.get_C());
            private_ops::sub_t<R,C>(*this, a, *this);
            return *this;
        }
        inline static_matrix<T,R,C,S>& operator-=(T a)
        {
            private_ops::sub_t<R, C>(*this, a, *this);
            return *this;
        }

        inline static_matrix<T,R,C,S> operator-()
        {
            static_matrix<T,R,C,S> result;
            private_ops::negate_t<R, C>(*this, result);
            return result;
        }
        inline static_matrix<T,R,C,S>& negate()
        {
            private_ops::negate_t<R, C>(*this, *this);
            return *this;
        }

        template<u8 C2, structural_ids::en S2>
        inline static_matrix<T,R,C2,S> operator*(const static_matrix<T,C,C2,S2>& a) const
        {
            static_matrix<T,R,C2> result;
            private_ops::mul_t<R,C,C2>(*this, a, result);
            return result;
        }
        template <structural_ids::en S2>
        inline dynamic_matrix<T,S> operator*(const dynamic_matrix<T,S2>& a) const
        {
            assert(C == a.get_R());
            dynamic_matrix<T> result(R, a.get_C());
            private_ops::mul(R, C, a.get_C(), *this, a, result);
            return result;
        }
        inline static_matrix<T,R,C,S> operator*(T a) const
        {
            static_matrix<T,R,C,S> result;
            private_ops::mul_t<R, C>(*this, a, result);
            return result;
        }
        inline static_matrix<T,R,C,S> operator/(T a) const
        {
            static_matrix<T,R,C,S> result;
            private_ops::mul_t<R, C>(*this, static_cast<T>(1)/a, result);
            return result;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,R,S>& operator*=(const static_matrix<T,R,R,S2>& a)
        {
            *this = this->operator*(a);
            return *this;
        }
        template <structural_ids::en S2>
        inline static_matrix<T,R,R,S>& operator*=(const dynamic_matrix<T,S2>& a)
        {
            assert(a.get_R() == a.get_C() && R == a.get_R());
            static_matrix<T,R,R> result;
            private_ops::mul_t<R,R,R>(*this, a, result);
            *this = result;
            return *this;
        }
        inline static_matrix<T,R,C,S>& operator*=(T a)
        {
            private_ops::mul_t<R,C>(*this, a, *this);
            return *this;
        }
        inline static_matrix<T,R,C,S>& operator/=(T a)
        {
            private_ops::mul_t<R,C>(*this, static_cast<T>(1)/a, *this);
            return *this;
        }
        inline static_matrix<T,R-1,1,structural_ids::normal> homogeneous_mul(const static_matrix<T,R-1,1,structural_ids::normal>& vec) const
        {
            BOOST_STATIC_ASSERT(R == C); // matrix should be square for vector transform operations
            static_matrix<T,R-1,1,structural_ids::normal> result;
            private_ops::homogeneous_mul(R, C, *this, vec, result);
            return result;
        }
        inline static_matrix<T,R-1,1,structural_ids::normal> homogeneous_mul(const dynamic_matrix<T,structural_ids::normal>& vec) const
        {
            BOOST_STATIC_ASSERT(R == C); // matrix should be square for vector transform operations
            assert(vec.get_R() == R && vec.get_C() == 1);
            static_matrix<T,R-1,1,structural_ids::normal> result;
            private_ops::homogeneous_mul(R, C, *this, vec, result);
            return result;
        }

        inline T det() const
        {
            BOOST_STATIC_ASSERT(R == C);
            return private_ops::det_t<R,S>::det<T>(*this);
        }

        inline static_matrix<T,C,R,S> transpose() const
        {
            static_matrix<T,C,R,S> result;
            private_ops::transpose_t<R,C>(*this, result);
            return result;
        }

        inline static_matrix<T,R,R,S> inverse() const
        {
            BOOST_STATIC_ASSERT(R == C);
            static_matrix<T,R,R,S> result;
            private_ops::inverse_t<R,S>::inverse(*this, result);
            return result;
        }

        inline static_matrix<T,R,R,S>& invert()
        {
            *this = inverse();
            return *this;
        }

        inline T norm() const
        {
            BOOST_STATIC_ASSERT(R == 1 || C == 1);
            T square_sum = private_ops::dot_prod_t<(R>C)?R:C>(*this, *this);
            return sqrt(square_sum);
        }
        inline static_matrix<T,R,C,S>& normalize(T target_norm = 1.)
        {
            T n = target_norm/norm();
            private_ops::mul_t<R,C>(*this, n, *this);
            return *this;
        }

        template <u8 R2, u8 C2>
        inline T dot_prod(const static_matrix<T,R2,C2,structural_ids::normal>& a) const
        {
            BOOST_STATIC_ASSERT((R == 1 || C == 1) && (R2 == 1 || C2 == 1)); // Make sure both operand are vectors
            BOOST_STATIC_ASSERT((R != 1 && R == R2) || (R != 1 && R == C2) || (C != 1 && C == R2) || (C != 1 && C == C2));
            return private_ops::dot_prod_t<(R>C)?R:C>(*this, a);
        }
        inline T dot_prod(const dynamic_matrix<T,structural_ids::normal>& a) const
        {
            BOOST_STATIC_ASSERT(R == 1 || C == 1); // Make sure both operand are vectors
            assert(a.get_R() == 1 || a.get_C() == 1);
            assert((R != 1 && R == a.get_R()) || (R != 1 && R == a.get_C()) || (C != 1 && C == a.get_R()) || (C != 1 && C == a.get_C()));
            return private_ops::dot_prod_t<(R>C)?R:C>(*this, a);
        }

        template <u8 R2, u8 C2>
        inline static_matrix<T,R,C,S> cross_prod(const static_matrix<T,R2,C2,structural_ids::normal>& a) const
        {
            BOOST_STATIC_ASSERT((R == 1 && C == 3) || (R == 3 && C == 1)); // Make sure both operands are 3D vectors
            BOOST_STATIC_ASSERT((R2 == 1 && C2 == 3) || (R2 == 3 && C2 == 1));
            static_matrix<T,R,C,S> result;
            private_ops::cross_prod(*this, a, result);
            return result;
        }
        inline static_matrix<T,R,C,S> cross_prod(const dynamic_matrix<T,structural_ids::normal>& a) const
        {
            BOOST_STATIC_ASSERT((R == 1 && C == 3) || (R == 3 && C == 1)); // Make sure both operands are 3D vectors
            assert((a.get_R() == 1 && a.get_C() == 3) || (a.get_R() == 3 && a.get_C() == 1));
            static_matrix<T,R,C,S> result;
            private_ops::cross_prod(*this, a, result);
            return result;
        }

        template <u8 R2, u8 C2>
        inline T angle_rad(const static_matrix<T,R2,C2,structural_ids::normal>& a) // get angle between 2 vectors
        {
            BOOST_STATIC_ASSERT((R == 1 || C == 1) && (R2 == 1 || C2 == 1)); // Make sure both operand are vectors
            BOOST_STATIC_ASSERT((R != 1 && R == R2) || (R != 1 && R == C2) || (C != 1 && C == R2) || (C != 1 && C == C2));
            return std::acos(min_t<T>(dot_prod(a)/(norm()*a.norm()), 1.));
        }
        inline T angle_rad(const dynamic_matrix<T,structural_ids::normal>& a) // get angle between 2 vectors
        {
            BOOST_STATIC_ASSERT(R == 1 || C == 1); // Make sure both operand are vectors
            assert((a.get_R() == 1 || a.get_C() == 1) && ((a.get_R() != 1 && a.get_R() == R) || (a.get_R() != 1 && a.get_R() == C) || (a.get_C() != 1 && a.get_C() == R) || (a.get_C() != 1 && a.get_C() == C)));
            return std::acos(min_t<T>(dot_prod(a)/(norm()*a.norm()), 1.));
        }
        template <u8 R2, u8 C2>
        inline T angle_deg(const static_matrix<T,R,C,structural_ids::normal>& a)
        {
            BOOST_STATIC_ASSERT(R == 1 || C == 1); // Make sure both operand are vectors
            return rad_to_deg<T>(angle_rad(a));
        }
        inline T angle_deg(const dynamic_matrix<T,structural_ids::normal>& a)
        {
            BOOST_STATIC_ASSERT(R == 1 || C == 1); // Make sure both operand are vectors
            return rad_to_deg<T>(angle_rad(a));
        }
    };

    // dynamic_matrix allocates its memory on the heap. use this version when you don't know both dimensions in advance.
    template <typename T, structural_ids::en S>
    class dynamic_matrix
    {
    public:
        typedef T T;
        dynamic_buffer<T,S> buf;
        inline const T get(u32 index) const { return buf.get(index); }
        inline const T get(u8 r, u8 c) const { return buf.get(r, c); }
        inline T& set(u32 index) { return buf.set(index); }
        inline T& set(u8 r, u8 c) { return buf.set(r, c); }

        inline u8 get_R() const { return buf.get_R(); }
        inline u8 get_C() const { return buf.get_C(); }
        dynamic_matrix(u8 R, u8 C) : buf(R, C) {}
        dynamic_matrix(u8 R, u8 C, T from) : buf(R, C, from) {}
        template <u8 R, u8 C, structural_ids::en S2>
        dynamic_matrix(const static_matrix<T,R,C,S2>& from) : buf(from.buf) { }
        template <structural_ids::en S2>
        dynamic_matrix(const dynamic_matrix<T,S2>& from) : buf(from.buf) { }
        dynamic_matrix(const dynamic_matrix<T,S>& from) : buf(from.buf) { }
        dynamic_matrix(T from) : buf(from) { }
        template <u8 R, u8 C, structural_ids::en S2>
        inline dynamic_matrix<T,S>& operator=(const static_matrix<T,R,C,S2>& from) { buf = from.buf; return *this; }
        template <structural_ids::en S2>
        inline dynamic_matrix<T,S>& operator=(const dynamic_matrix<T,S2>& from) { buf = from.buf; return *this; }
        inline dynamic_matrix<T,S>& operator=(T from) { buf = from; return *this; }

        inline void init(T v0)
        {
            assert(R > 0 && C == 1);
            set(0) = v0;
            for (u8 i = 1; i < R; i++) set(i) = 0;
        }
        inline void init(T v0, T v1)
        {
            assert(R > 1 && C == 1);
            set(0) = v0;
            set(1) = v1;
            for (u8 i = 2; i < R; i++) set(i) = 0;
        }
        inline void init(T v0, T v1, T v2)
        {
            assert(R > 2 && C == 1);
            set(0) = v0;
            set(1) = v1;
            set(2) = v2;
            for (u8 i = 3; i < R; i++) set(i) = 0;
        }
        inline void init(T v0, T v1, T v2, T v3)
        {
            assert((R > 3) && (C == 1));
            set(0) = v0;
            set(1) = v1;
            set(2) = v2;
            set(3) = v3;
            for (u8 i = 4; i < R; i++) set(i) = 0;
        }

        inline void identity()
        {
            assert(get_R() == get_C());
            private_ops::identity(get_R(), S, *this);
        }

        inline dynamic_matrix<T,S> operator+(const dynamic_matrix<T>& a) const
        {
            assert(get_R() == a.get_R() && get_C() == a.get_C());
            dynamic_matrix<T> result(get_R(), get_C());
            private_ops::add(get_R(), get_C(), *this, a, result);
            return result;
        }
        template <u8 R, u8 C>
        inline dynamic_matrix<T,S> operator+(const static_matrix<T,R,C>& a) const
        {
            assert(get_R() == R && get_C() == C);
            dynamic_matrix<T> result(R, C);
            private_ops::add_t<R,C>(*this, a, result);
            return result;
        }
        inline dynamic_matrix<T,S> operator+(T a) const
        {
            dynamic_matrix<T> result(get_R(), get_C());
            private_ops::add(get_R(), get_C(), *this, a, result);
            return result;
        }
        inline dynamic_matrix<T,S>& operator+=(const dynamic_matrix<T>& a)
        {
            assert(get_R() == a.get_R() && get_C() == a.get_C());
            private_ops::add(get_R(), get_C(), *this, a, *this);
            return *this;
        }
        template <u8 R, u8 C>
        inline dynamic_matrix<T,S>& operator+=(const static_matrix<T,R,C>& a)
        {
            assert(get_R() == R && get_C() == C);
            private_ops::add_t<R,C>(*this, a, *this);
            return *this;
        }
        inline dynamic_matrix<T,S>& operator+=(T a)
        {
            private_ops::add(get_R(), get_C(), *this, a, *this);
            return *this;
        }

        inline dynamic_matrix<T,S> operator-(const dynamic_matrix<T>& a) const
        {
            assert(get_R() == a.get_R() && get_C() == a.get_C());
            dynamic_matrix<T> result(get_R(), get_C());
            private_ops::sub(get_R(), get_C(), *this, a, result);
            return result;
        }
        template <u8 R, u8 C>
        inline dynamic_matrix<T,S> operator-(const static_matrix<T,R,C>& a) const
        {
            assert(get_R() == R && get_C() == C);
            dynamic_matrix<T> result(R, C);
            private_ops::sub_t<R,C>(*this, a, result);
            return result;
        }
        inline dynamic_matrix<T,S> operator-(T a) const
        {
            dynamic_matrix<T> result(get_R(), get_C());
            private_ops::sub(get_R(), get_C(), *this, a, result);
            return result;
        }
        inline dynamic_matrix<T,S>& operator-=(const dynamic_matrix<T>& a)
        {
            assert(get_R() == a.get_R() && get_C() == a.get_C());
            private_ops::sub(get_R(), get_C(), *this, a, *this);
            return *this;
        }
        template <u8 R, u8 C>
        inline dynamic_matrix<T,S>& operator-=(const static_matrix<T,R,C>& a)
        {
            assert(get_R() == R && get_C() == C);
            private_ops::sub_t<R,C>(*this, a, *this);
            return *this;
        }
        inline dynamic_matrix<T,S>& operator-=(T a)
        {
            private_ops::sub(get_R(), get_C(), *this, a, *this);
            return *this;
        }

        inline dynamic_matrix<T,S> operator-()
        {
            dynamic_matrix<T,S> result;
            private_ops::negate(R, C, *this, result);
            return result;
        }
        inline dynamic_matrix<T,S>& negate()
        {
            private_ops::negate(R, C, *this, *this);
            return *this;
        }

        inline dynamic_matrix<T,S> operator*(const dynamic_matrix<T>& a) const
        {
            assert(get_C() == a.get_R());
            dynamic_matrix<T> result(get_R(), a.get_C());
            private_ops::mul(get_R(), get_C(), a.get_C(), *this, a, result);
            return result;
        }
        template <u8 R, u8 C>
        inline dynamic_matrix<T,S> operator*(const static_matrix<T,R,C>& a) const
        {
            assert(get_C() == R);
            dynamic_matrix<T> result(get_R(), C);
            private_ops::mul(get_R(), R, C, *this, a, result);
            return result;
        }
        inline dynamic_matrix<T,S> operator*(T a) const
        {
            dynamic_matrix<T> result(get_R(), get_C());
            private_ops::mul(get_R(), get_C(), *this, a, result);
            return result;
        }
        inline dynamic_matrix<T,S> operator/(T a) const
        {
            dynamic_matrix<T> result(get_R(), get_C());
            private_ops::mul(get_R(), get_C(), *this, static_cast<T>(1)/a, result);
            return result;
        }
        inline dynamic_matrix<T,S>& operator*=(const dynamic_matrix<T>& a)
        {
            assert(get_R() == get_C() && a.get_R() == a.get_C() && get_R() == a.get_R());
            *this = this->operator*(a);
            return *this;
        }
        template <u8 R>
        inline dynamic_matrix<T,S>& operator*=(const static_matrix<T,R,R>& a)
        {
            assert(get_R() == R && get_R() == get_C());
            *this = this->operator*(a);
            return *this;
        }
        inline dynamic_matrix<T,S>& operator*=(T a)
        {
            private_ops::mul(get_R(), get_C(), *this, a, *this);
            return *this;
        }
        inline dynamic_matrix<T,S>& operator/=(T a)
        {
            private_ops::mul(get_R(), get_C(), *this, static_cast<T>(1)/a, *this);
            return *this;
        }
        template <u8 R2>
        inline dynamic_matrix<T,structural_ids::normal> homogeneous_mul(const static_matrix<T,R2,1,structural_ids::normal>& vec) const
        {
            assert(get_R() == get_C() && (vec.get_R() - 1) == R2); // matrix should be square for vector transform operations
            dynamic_matrix<T,structural_ids::normal> result;
            private_ops::homogeneous_mul(get_R(), get_C(), *this, vec, result);
            return result;
        }
        inline dynamic_matrix<T,structural_ids::normal> homogeneous_mul(const dynamic_matrix<T,structural_ids::normal>& vec) const
        {
            assert(get_R() == get_C() && vec.get_R() == get_R() && vec.get_C() == 1); // matrix should be square for vector transform operations
            dynamic_matrix<T,structural_ids::normal> result;
            private_ops::homogeneous_mul(get_R(), get_C(), *this, vec, result);
            return result;
        }

        inline T det() const
        {
            assert(get_R() == get_C());
            return private_ops::det<T>(get_R(), S, *this);
        }

        inline dynamic_matrix<T,S> transpose() const
        {
            dynamic_matrix<T,S> result(get_C(), get_R());
            private_ops::transpose(get_R(), get_C(), *this, result);
            return result;
        }

        inline dynamic_matrix<T,S> inverse() const
        {
            assert(get_R() == get_C());
            dynamic_matrix<T,S> result(get_R(), get_R());
            private_ops::inverse(get_R(), S, *this, result);
            return result;
        }

        inline dynamic_matrix<T,S>& invert()
        {
            *this = inverse();
            return *this;
        }

        inline T norm() const
        {
            assert(get_R() == 1 || get_C() == 1); // Make sure this is a pointer
            T square_sum = private_ops::dot_prod(max_t(get_R(), get_C()), *this, *this);
            return sqrt(square_sum);
        }
        inline dynamic_matrix<T,S>& normalize(T target_norm = 1.)
        {
            T n = target_norm/norm();
            private_ops::mul(get_R(), get_C(), *this, n, *this);
            return *this;
        }

        template <u8 R2, u8 C2>
        inline T dot_prod(const static_matrix<T,R2,C2,structural_ids::normal>& a) const
        {
            BOOST_STATIC_ASSERT((R2 == 1 || C2 == 1)); // Make sure operand is a vector
            assert((get_R() == 1 || get_C() == 1)); // Make sure this is a vector
            assert((get_R() != 1 && get_R() == R2) || (get_R() != 1 && get_R() == C2) || (get_C() != 1 && get_C() == R2) || (get_C() != 1 && get_C() == C2));
            return private_ops::dot_prod_t<(R2>C2)?R2:C2>(*this, a);
        }
        inline T dot_prod(const dynamic_matrix<T,structural_ids::normal>& a) const
        {
            BOOST_STATIC_ASSERT(R == 1 || C == 1); // Make sure both operand are vectors
            assert(a.get_R() == 1 || a.get_C() == 1);
            assert((R != 1 && R == a.get_R()) || (R != 1 && R == a.get_C()) || (C != 1 && C == a.get_R()) || (C != 1 && C == a.get_C()));
            return private_ops::dot_prod(max_t<T>(get_R(), get_C()), *this, a);
        }

        template <u8 R2, u8 C2>
        inline dynamic_matrix<T,S> cross_prod(const static_matrix<T,R2,C2,structural_ids::normal>& a) const
        {
            BOOST_STATIC_ASSERT((R2 == 1 && C2 == 3) || (R2 == 3 && C2 == 1));
            assert((get_R() == 1 && get_C() == 3) || (get_R() == 3 && get_C() == 1)); // Make sure both operands are 3D vectors
            dynamic_matrix<T,S> result;
            private_ops::cross_prod(*this, a, result);
            return result;
        }
        inline dynamic_matrix<T,S> cross_prod(const dynamic_matrix<T,structural_ids::normal>& a) const
        {
            assert((get_R() == 1 && get_C() == 3) || (get_R() == 3 && get_C() == 1)); // Make sure both operands are 3D vectors
            assert((a.get_R() == 1 && a.get_C() == 3) || (a.get_R() == 3 && a.get_C() == 1));
            dynamic_matrix<T,S> result;
            private_ops::cross_prod(*this, a, result);
            return result;
        }

        template <u8 R2, u8 C2>
        inline T angle_rad(const static_matrix<T,R2,C2,structural_ids::normal>& a) // get angle between 2 vectors
        {
            BOOST_STATIC_ASSERT(R == 1 || C == 1); // Make sure both operand are vectors
            assert(get_R() == 1 || get_C() == 1);
            assert((get_R() != 1 && get_R() == R) || (get_R() != 1 && get_R() == C) || (get_C() != 1 && get_C() == R) || (get_C() != 1 && get_C() == C));
            return acos(min_t<T>(dot_prod(a)/(norm()*a.norm()), 1.));
        }
        inline T angle_rad(const dynamic_matrix<T,structural_ids::normal>& a) // get angle between 2 vectors
        {
            assert(get_R() == 1 || get_C() == 1); // Make sure both operand are vectors
            assert((a.get_R() == 1 || a.get_C() == 1));
            assert((get_R() != 1 && get_R() == a.get_R()) || (get_R() != 1 && get_R() == a.get_C()) || (get_C() != 1 && get_C() == a.get_R()) || (get_C() != 1 && get_C() == a.get_C()));
            return acos(min_t<T>(dot_prod(a)/(norm()*a.norm()), 1.));
        }
        template <u8 R2, u8 C2>
        inline T angle_deg(const static_matrix<T,R2,C2,structural_ids::normal>& a)
        {
            BOOST_STATIC_ASSERT(R2 == 1 || C2 == 1); // Make sure operand is a vector
            return rad_to_deg<T>(angle_rad(a));
        }
        inline T angle_deg(const dynamic_matrix<T,structural_ids::normal>& a)
        {
            return rad_to_deg<T>(angle_rad(a));
        }
    };

    // matrix statically chooses between static and dynamic implementation depending on R and C : if any is 0, then dynamic is chosen.
    template <typename T, u8 R = 0, u8 C = 0, structural_ids::en S = structural_ids::normal>
    struct matrix : public static_matrix<T,R,C,S>
    {
        matrix() {}
        template <structural_ids::en S2>
        matrix(const static_matrix<T,R,C,S2>& from) : static_matrix<T,R,C,S>(from) { }
        template <structural_ids::en S2>
        matrix(const dynamic_matrix<T,S2>& from) : static_matrix<T,R,C,S>(from) { }
        matrix(T from) : static_matrix<T,R,C,S>(from) {}
        template <structural_ids::en S2>
        inline matrix<T,R,C,S>& operator=(const static_matrix<T,R,C,S2>& from) { static_cast< static_matrix<T,R,C,S>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline matrix<T,R,C,S>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< static_matrix<T,R,C,S>* >(this)->operator=(from); return *this; }
        inline matrix<T,R,C,S>& operator=(T from) { static_cast< static_matrix<T,R,C,S>* >(this)->operator=(from); return *this; }
    };
    template <typename T, u8 R, structural_ids::en S>
    struct matrix<T,R,0,S> : public dynamic_matrix<T,S>
    {
        matrix(u8 C_init = 1) : dynamic_matrix<T,S>(R, C_init) {}
        template <u8 C2, structural_ids::en S2>
        matrix(const static_matrix<T,R,C2,S2>& from) : dynamic_matrix<T,S>(from) { }
        matrix(const dynamic_matrix<T>& from) : dynamic_matrix<T,S>(from) { }
        matrix(u8 C_init, T from) : dynamic_matrix<T,S>(R, C_init, from) {}
        template <u8 C2, structural_ids::en S2>
        inline matrix<T,R,0,S>& operator=(const static_matrix<T,R,C2,S2>& from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline matrix<T,R,0,S>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
        inline matrix<T,R,0,S>& operator=(T from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
    };
    template <typename T, u8 C, structural_ids::en S>
    struct matrix<T,0,C,S> : public dynamic_matrix<T,S>
    {
        matrix(u8 R_init = 1) : dynamic_matrix<T>(R_init, C) {}
        template <u8 R2, structural_ids::en S2>
        matrix(const static_matrix<T,R2,C,S2>& from) : dynamic_matrix<T,S>(from) { }
        template <structural_ids::en S2>
        matrix(const dynamic_matrix<T,S2>& from) : dynamic_matrix<T,S>(from) { }
        matrix(u8 R_init, T from) : dynamic_matrix<T>(R_init, C, from) {}
        template <u8 R2, structural_ids::en S2>
        inline matrix<T,0,C,S>& operator=(const static_matrix<T,R2,C,S2>& from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline matrix<T,0,C,S>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
        inline matrix<T,0,C,S>& operator=(T from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
    };
    template <typename T, structural_ids::en S>
    struct matrix<T,0,0,S> : public dynamic_matrix<T,S>
    {
        matrix(u8 R_init = 1, u8 C_init = 1) : dynamic_matrix<T,S>(R_init, C_init) {}
        template <u8 R2, u8 C2, structural_ids::en S2>
        matrix(const static_matrix<T,R2,C2,S2>& from) : dynamic_matrix<T,S>(from) { }
        template <structural_ids::en S2>
        matrix(const dynamic_matrix<T,S2>& from) : dynamic_matrix<T,S>(from) { }
        matrix(u8 R_init, u8 C_init, T from) : dynamic_matrix<T,S>(R_init, C_init, from) {}
        template <u8 R2, u8 C2, structural_ids::en S2>
        inline matrix<T,0,0,S>& operator=(const static_matrix<T,R2,C2,S2>& from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline matrix<T,0,0,S>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
        inline matrix<T,0,0,S>& operator=(T from) { static_cast< dynamic_matrix<T,S>* >(this)->operator=(from); return *this; }
    };

    // column vector shortcut : matrix<R,1>
    template <typename T, u8 R = 0>
    struct vector : public static_matrix<T,R,1,structural_ids::normal>
    {
        vector() {}
        template <structural_ids::en S2>
        vector(const static_matrix<T,R,1,S2>& from) : static_matrix<T,R,1,structural_ids::normal>(from) { }
        template <structural_ids::en S2>
        vector(const dynamic_matrix<T,S2>& from) : static_matrix<T,R,1,structural_ids::normal>(from) { }
        vector(T from) : static_matrix<T,R,1,structural_ids::normal>(from) {}
        template <structural_ids::en S2>
        inline vector<T,R>& operator=(const static_matrix<T,R,1,S2>& from) { static_cast< static_matrix<T,R,1,structural_ids::normal>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline vector<T,R>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< static_matrix<T,R,1,structural_ids::normal>* >(this)->operator=(from); return *this; }
        inline vector<T,R>& operator=(T from) { static_cast< static_matrix<T,R,1,structural_ids::normal>* >(this)->operator=(from); return *this; }
    };
    template <typename T>
    struct vector<T,0> : public dynamic_matrix<T,structural_ids::normal>
    {
        vector() {}
        template <u8 R2, structural_ids::en S2>
        vector(const static_matrix<T,R2,1,S2>& from) : dynamic_matrix<T,structural_ids::normal>(from) { }
        template <structural_ids::en S2>
        vector(const dynamic_matrix<T,S2>& from) : dynamic_matrix<T,structural_ids::normal>(from) { }
        vector(T from) : dynamic_matrix<T,structural_ids::normal>(from) {}
        template <u8 R2, structural_ids::en S2>
        inline vector<T,0>& operator=(const static_matrix<T,R2,1,S2>& from) { static_cast< dynamic_matrix<T,structural_ids::normal>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline vector<T,0>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< dynamic_matrix<T,structural_ids::normal>* >(this)->operator=(from); return *this; }
        inline vector<T,0>& operator=(T from) { static_cast< dynamic_matrix<T,structural_ids::normal>* >(this)->operator=(from); return *this; }
    };
    // line vector shortcut : matrix<1,C>
    template <typename T, u8 C = 0>
    struct l_vector : public static_matrix<T,1,C,structural_ids::normal>
    {
        l_vector() {}
        template <structural_ids::en S2>
        l_vector(const static_matrix<T,1,C,S2>& from) : static_matrix<T,1,C,structural_ids::normal>(from) { }
        template <structural_ids::en S2>
        l_vector(const dynamic_matrix<T,S2>& from) : static_matrix<T,1,C,structural_ids::normal>(from) { }
        l_vector(T from) : static_matrix<T,1,C,structural_ids::normal>(from) {}
        template <structural_ids::en S2>
        inline l_vector<T,C>& operator=(const static_matrix<T,1,C,S2>& from) { static_cast< static_matrix<T,1,C,structural_ids::normal>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline l_vector<T,C>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< static_matrix<T,1,C,structural_ids::normal>* >(this)->operator=(from); return *this; }
        inline l_vector<T,C>& operator=(T from) { static_cast< static_matrix<T,1,C,structural_ids::normal>* >(this)->operator=(from); return *this; }
    };
    template <typename T>
    struct l_vector<T,0> : public dynamic_matrix<T,structural_ids::normal>
    {
        l_vector() {}
        template <u8 C2, structural_ids::en S2>
        l_vector(const static_matrix<T,1,C2,S2>& from) : dynamic_matrix<T,structural_ids::normal>(from) { }
        template <structural_ids::en S2>
        l_vector(const dynamic_matrix<T,S2>& from) : dynamic_matrix<T,structural_ids::normal>(from) { }
        l_vector(T from) : dynamic_matrix<T,structural_ids::normal>(from) {}
        template <u8 C2, structural_ids::en S2>
        inline l_vector<T,0>& operator=(const static_matrix<T,1,C2,S2>& from) { static_cast< dynamic_matrix<T,structural_ids::normal>* >(this)->operator=(from); return *this; }
        template <structural_ids::en S2>
        inline l_vector<T,0>& operator=(const dynamic_matrix<T,S2>& from) { static_cast< dynamic_matrix<T,structural_ids::normal>* >(this)->operator=(from); return *this; }
        inline l_vector<T,0>& operator=(T from) { static_cast< dynamic_matrix<T,structural_ids::normal>* >(this)->operator=(from); return *this; }
    };

    // C-style operations with specified return type
    // TODO : sub, negate, crossprod, add/sub/mul/div with constant, transpose, invert, etc. all ops returning a matrix
    template <typename T, u8 R1, u8 C1>
    inline void add(const static_matrix<T,R1,C1>& a, const static_matrix<T,R1,C1>& b, static_matrix<T,R1,C1>& c)
    {
        private_ops::add_t<R1,C1>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void add(const dynamic_matrix<T>& a, const static_matrix<T,R1,C1>& b, static_matrix<T,R1,C1>& c)
    {
        assert(R1 == a.get_R() && C1 == a.get_C());
        private_ops::add_t<R1,C2>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void add(const static_matrix<T,R1,C1>& a, const dynamic_matrix<T>& b, static_matrix<T,R1,C1>& c)
    {
        assert(R1 == b.get_R() && C1 == b.get_C());
        private_ops::add_t<R1,C1>(a, b, c);
    }
    template <typename T, u8 R3, u8 C3>
    inline void add(const dynamic_matrix<T>& a, const dynamic_matrix<T>& b, static_matrix<T,R3,C3>& c)
    {
        assert(a.get_R() == R3 && b.get_R() == R3 && a.get_C() == C3 && b.get_C() == C3);
        private_ops::add_t<R3,C3>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void add(const static_matrix<T,R1,C1>& a, const static_matrix<T,R1,C1>& b, dynamic_matrix<T>& c)
    {
        assert(R1 == c.get_R() && C1 == c.get_C());
        private_ops::add_t<R1,C1>(a, b, c);
    }
    template <typename T, u8 R2, u8 C2>
    inline void add(const dynamic_matrix<T>& a, const static_matrix<T,R2,C2>& b, dynamic_matrix<T>& c)
    {
        assert(a.get_R() == R2 && R2 == c.get_R() && a.get_C() == C2 && C2 == c.get_C());
        private_ops::add_t<R2, C2>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void add(const static_matrix<T,R1,C1>& a, const dynamic_matrix<T>& b, dynamic_matrix<T>& c)
    {
        assert(R1 == b.get_R() && R1 == c.get_R() && C1 == b.get_C() && C1 == c.get_C());
        private_ops::add_t<R1, C1>(a, b, c);
    }
    template <typename T>
    inline void add(const dynamic_matrix<T>& a, const dynamic_matrix<T>& b, dynamic_matrix<T>& c)
    {
        assert(a.get_R() == b.get_R() && a.get_R() == c.get_R() && a.get_C() == b.get_C() && a.get_C() == c.get_C());
        private_ops::add(a.get_R(), a.get_C(), a, b, c);
    }

    template <typename T, u8 R1, u8 C1>
    inline void sub(const static_matrix<T,R1,C1>& a, const static_matrix<T,R1,C1>& b, static_matrix<T,R1,C1>& c)
    {
        private_ops::sub_t<R1,C1>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void sub(const dynamic_matrix<T>& a, const static_matrix<T,R1,C1>& b, static_matrix<T,R1,C1>& c)
    {
        assert(R1 == a.get_R() && C1 == a.get_C());
        private_ops::sub_t<R1,C2>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void sub(const static_matrix<T,R1,C1>& a, const dynamic_matrix<T>& b, static_matrix<T,R1,C1>& c)
    {
        assert(R1 == b.get_R() && C1 == b.get_C());
        private_ops::sub_t<R1,C1>(a, b, c);
    }
    template <typename T, u8 R3, u8 C3>
    inline void sub(const dynamic_matrix<T>& a, const dynamic_matrix<T>& b, static_matrix<T,R3,C3>& c)
    {
        assert(a.get_R() == R3 && b.get_R() == R3 && a.get_C() == C3 && b.get_C() == C3);
        private_ops::sub_t<R3,C3>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void sub(const static_matrix<T,R1,C1>& a, const static_matrix<T,R1,C1>& b, dynamic_matrix<T>& c)
    {
        assert(R1 == c.get_R() && C1 == c.get_C());
        private_ops::sub_t<R1,C1>(a, b, c);
    }
    template <typename T, u8 R2, u8 C2>
    inline void sub(const dynamic_matrix<T>& a, const static_matrix<T,R2,C2>& b, dynamic_matrix<T>& c)
    {
        assert(a.get_R() == R2 && R2 == c.get_R() && a.get_C() == C2 && C2 == c.get_C());
        private_ops::sub_t<R2, C2>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1>
    inline void sub(const static_matrix<T,R1,C1>& a, const dynamic_matrix<T>& b, dynamic_matrix<T>& c)
    {
        assert(R1 == b.get_R() && R1 == c.get_R() && C1 == b.get_C() && C1 == c.get_C());
        private_ops::sub_t<R1, C1>(a, b, c);
    }
    template <typename T>
    inline void sub(const dynamic_matrix<T>& a, const dynamic_matrix<T>& b, dynamic_matrix<T>& c)
    {
        assert(a.get_R() == b.get_R() && a.get_R() == c.get_R() && a.get_C() == b.get_C() && a.get_C() == c.get_C());
        private_ops::sub(a.get_R(), a.get_C(), a, b, c);
    }

    template <typename T, u8 R1, u8 C1R2, u8 C2>
    inline void mul(const static_matrix<T,R1,C1R2>& a, const static_matrix<T,C1R2,C2>& b, static_matrix<T,R1,C2>& c)
    {
        private_ops::mul_t<R1,C1R2,C2>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1R2, u8 C2>
    inline void mul(const dynamic_matrix<T>& a, const static_matrix<T,C1R2,C2>& b, static_matrix<T,R1,C2>& c)
    {
        assert(R1 == a.get_R() && C1R2 == a.get_C());
        private_ops::mul_t<R1,C1R2,C2>(a, b, c);
    }
    template <typename T, u8 R1, u8 C1R2, u8 C2>
    inline void mul(const static_matrix<T,R1,C1R2>& a, const dynamic_matrix<T>& b, static_matrix<T,R1,C2>& c)
    {
        assert(C1R2 == b.get_R() && b.get_C() == C2);
        private_ops::mul_t<R1,C1R2,C2>(a, b, c);
    }
    template <typename T, u8 R1, u8 C2>
    inline void mul(const dynamic_matrix<T>& a, const dynamic_matrix<T>& b, static_matrix<T,R1,C2>& c)
    {
        assert(a.get_R() == R1 && a.get_C() == b.get_R() && C2 == b.get_C());
        private_ops::mul(R1, a.get_C(), C2, a, b, c);
    }
    template <typename T, u8 R1, u8 C1R2, u8 C2>
    inline void mul(const static_matrix<T,R1,C1R2>& a, const static_matrix<T,C1R2,C2>& b, dynamic_matrix<T>& c)
    {
        assert(R1 == c.get_R() && C2 == c.get_C());
        private_ops::mul_t<R1,C1R2,C2>(a, b, c);
    }
    template <typename T, u8 C1R2, u8 C2>
    inline void mul(const dynamic_matrix<T>& a, const static_matrix<T,C1R2,C2>& b, dynamic_matrix<T>& c)
    {
        assert(a.get_R() == c.get_R() && a.get_C() == C1R2 && C2 == c.get_C());
        private_ops::mul(a.get_R(), C1R2, C2, a, b, c);
    }
    template <typename T, u8 R1, u8 C1R2>
    inline void mul(const static_matrix<T,R1,C1R2>& a, const dynamic_matrix<T>& b, dynamic_matrix<T>& c)
    {
        assert(R1 == c.get_R() && C1R2 == b.get_R() && b.get_C() == c.get_C());
        private_ops::mul(R1, C1R2, b.get_C(), a, b, c);
    }
    template <typename T>
    inline void mul(const dynamic_matrix<T>& a, const dynamic_matrix<T>& b, dynamic_matrix<T>& c)
    {
        assert(a.get_R() == c.get_R() && a.get_C() == b.get_R() && b.get_C() == c.get_C());
        private_ops::mul(a.get_R(), a.get_C(), b.get_C(), a, b, c);
    }

    namespace private_ops // do not use : dimensions not checked. use public entry points instead
    {
        // all unchecked ops : dimensions are already checked at this point
        template <u8 R, u8 C, typename M1, typename M2, typename M3>
        inline void add_t(const M1& a, const M2& b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) + b.get(i);
        }
        template <u8 R, u8 C, typename M1, typename M3>
        inline void add_t(const M1& a, typename M1::T b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) + b;
        }
        template <typename M1, typename M2, typename M3>
        inline void add(u8 R, u8 C, const M1& a, const M2& b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) + b.get(i);
        }
        template <typename M1, typename M3>
        inline void add(u8 R, u8 C, const M1& a, typename M1::T b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) + b;
        }
        template <u8 R, u8 C, typename M1, typename M2, typename M3>
        inline void sub_t(const M1& a, const M2& b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) - b.get(i);
        }
        template <u8 R, u8 C, typename M1, typename M3>
        inline void sub_t(const M1& a, typename M1::T b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) - b;
        }
        template <u8 R, u8 C, typename M1, typename M2>
        inline void negate_t(const M1& a, M2& b)
        {
            for (u8 i = 0; i < R*C; i++) b.set(i) = -a.get(i);
        }
        template <typename M1, typename M2, typename M3>
        inline void sub(u8 R, u8 C, const M1& a, const M2& b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) - b.get(i);
        }
        template <typename M1, typename M3>
        inline void sub(u8 R, u8 C, const M1& a, typename M1::T b, M3& c)
        {
            for (u8 i = 0; i < R*C; i++) c.set(i) = a.get(i) - b;
        }
        template <typename M1, typename M2>
        inline void negate(u8 R, u8 C, const M1& a, M2& b)
        {
            for (u8 i = 0; i < R*C; i++) b.set(i) = -a.get(i);
        }
        template <u8 R1, u8 C1R2, u8 C2, typename M1, typename M2, typename M3>
        inline void mul_t(const M1& a, const M2& b, M3& c)
        {
            for (u8 i = 0; i < R1; i++)
            {
                for (u8 j = 0; j < C2; j++)
                {
                    c.set(i, j) = a.get(i, 0) * b.get(0, j);
                    for (u8 k = 1; k < C1R2; k++)
                        c.set(i, j) += a.get(i, k) * b.get(k, j);
                }
            }
        }
        template <u8 R1, u8 C1, typename M1, typename M3>
        inline void mul_t(const M1& a, typename M1::T b, M3& c)
        {
            for (u8 i = 0; i < R1*C1; i++)
                c.set(i) = a.get(i) * b;
        }
        template <typename M1, typename M2, typename M3>
        inline void mul(u8 R1, u8 C1R2, u8 C2, const M1& a, const M2& b, M3& c)
        {
            for (u8 i = 0; i < R1; i++)
            {
                for (u8 j = 0; j < C2; j++)
                {
                    c.set(i, j) = a.get(i, 0) * b.get(0, j);
                    for (u8 k = 1; k < C1R2; k++)
                        c.set(i, j) += a.get(i, k) * b.get(k, j);
                }
            }
        }
        template <typename M1, typename M3>
        inline void mul(u8 R1, u8 C1, const M1& a, typename M1::T b, M3& c)
        {
            for (u8 i = 0; i < R1*C1; i++)
                c.set(i) = a.get(i) * b;
        }
        template <u8 R, typename M1, typename M2>
        inline typename M1::T dot_prod_t(const M1& a, const M2& b)
        {
            typename M1::T accum = 0;
            for (u8 i = 0; i < R; i++) accum += a.get(i) * b.get(i);
            return accum;
        }
        template <typename M1, typename M2>
        inline typename M1::T dot_prod(u8 R, const M1& a, const M2& b)
        {
            typename M1::T accum = 0;
            for (u8 i = 0; i < R; i++) accum += a.get(i) * b.get(i);
            return accum;
        }
        template <typename M1, typename M2, typename M3>
        inline void cross_prod(const M1& a, const M2& b, M3& c)
        {
            c.set(0) = a.get(1)*b.get(2) - a.get(2)*b.get(1);
            c.set(1) = a.get(2)*b.get(0) - a.get(0)*b.get(2);
            c.set(2) = a.get(0)*b.get(1) - a.get(1)*b.get(0);
        }
        template <typename M1, typename M2, typename M3>
        inline void homogeneous_mul(u8 R, u8 C, const M1& mat, const M2& vec, M3& res)
        {
            for (u8 i = 0; i < R - 1; i++)
            {
                res.set(i) = mat.get(i, C - 1);
                for (u8 j = 0; j < C - 1; j++)
                    res.set(i) += mat.get(i, j) * vec.get(j);
            }
        }
        // class used since we need partial specialization of template functions, which is not allowed in C++ 
        template <u8 R, structural_ids::en S> struct identity_t {
            template <typename M1> static inline void identity(M1& m)
            {
                for (u8 i = 0; i < R; i++)
                    for (u8 j = 0; j < R; j++)
                        m.set(i, j) = (i == j) ? static_cast<typename M1::T>(1) : static_cast<typename M1::T>(0);
            }
        };
        template <u8 R> struct identity_t<R, structural_ids::diagonal> {
            template <typename M1> static inline void identity(M1& m)
            {
                for (u8 i = 0; i < R; i++) m.set(i, i) = static_cast<typename M1::T>(1);
            }
        };
        template <typename M1>
        inline void identity(u8 R, structural_ids::en S, M1& m)
        {
            switch (S)
            {
            case structural_ids::diagonal:
                for (u8 i = 0; i < R; i++) m.set(i, i) = static_cast<typename M1::T>(1);
                break;
            default:
                for (u8 i = 0; i < R; i++)
                    for (u8 j = 0; j < R; j++)
                        m.set(i, j) = (i == j) ? static_cast<typename M1::T>(1) : static_cast<typename M1::T>(0);
                break;
            }
        }

        template <u8 R, structural_ids::en S> struct det_t {
            template <typename T, typename M1> static inline T det(const M1& m) { assert(false); } // unimplemented for large R yet. look up LU decompositions on Wikipedia
        };
        template <structural_ids::en S> struct det_t<1, S> {
            template <typename T, typename M1> static inline T det(const M1& m) { return m.get(0); }
        };
        template <> struct det_t<2, structural_ids::normal> {
            template <typename T, typename M1> static inline T det(const M1& m) {
                return m.get(0, 0) * m.get(1, 1) - m.get(0, 1) * m.get(1, 0);
            }
        };
        template <> struct det_t<2, structural_ids::diagonal> {
            template <typename T, typename M1> static inline T det(const M1& m) {
                return m.get(0, 0) * m.get(1, 1);
            }
        };
        template <> struct det_t<3, structural_ids::normal> {
            template <typename T, typename M1> static inline T det(const M1& m) {
                return m.get(0, 0) * (m.get(1, 1) * m.get(2, 2) - m.get(1, 2) * m.get(2, 1)) -
                       m.get(0, 1) * (m.get(1, 0) * m.get(2, 2) - m.get(1, 2) * m.get(2, 0)) +
                       m.get(0, 2) * (m.get(1, 0) * m.get(2, 1) - m.get(1, 1) * m.get(2, 0));
            }
        };
        template <> struct det_t<3, structural_ids::diagonal> {
            template <typename T, typename M1> static inline T det(const M1& m) {
                return m.get(0, 0) * m.get(1, 1) * m.get(2, 2);
            }
        };
        template <typename M1>
        inline typename M1::T det(u8 R, structural_ids::en S, const M1& m)
        {
            switch (R)
            {
            case 1: return get(0);
            case 2:
                if (structural_ids::normal == S) return get(0, 0) * get(1, 1) - get(0, 1) * get(1, 0);
                else if (structural_ids::diagonal == S) return get(0, 0) * get(1, 1);
            case 3:
                if (structural_ids::normal == S) return get(0, 0) * (get(1, 1) * get(2, 2) - get(1, 2) * get(2, 1)) -
                                                        get(0, 1) * (get(1, 0) * get(2, 2) - get(1, 2) * get(2, 0)) +
                                                        get(0, 2) * (get(1, 0) * get(2, 1) - get(1, 1) * get(2, 0));
                else if (structural_ids::diagonal == S) return get(0, 0) * get(1, 1) * get(2, 2);
            }
            assert(false); // unimplemented. look up LU decompositions on Wikipedia
        }
        template <u8 R, u8 C, typename M1, typename M2>
        inline void transpose_t(const M1& a, M2& b)
        {
            for (u8 i = 0; i < R; i++)
                for (u8 j = 0; j < C; j++)
                    b.set(j, i) = a.get(i, j);
        }
        template <typename M1, typename M2>
        inline void transpose(u8 R, u8 C, const M1& a, M2& b)
        {
            for (u8 i = 0; i < R; i++)
                for (u8 j = 0; j < C; j++)
                    b.set(j, i) = a.get(i, j);
        }
        template <u8 R, structural_ids::en S> struct inverse_t {
            template <typename M1> static inline void inverse(const M1& a, M1& b) { assert(false); } // unimplemented for large R yet. det + cofactor matrix may be way to go.
        };
        template <structural_ids::en S> struct inverse_t<1,S> {
            template <typename M1> static inline void inverse(const M1& a, M1& b) { b.set(0) = 1. / a.get(0); }
        };
        template <> struct inverse_t<2,structural_ids::normal> {
            template <typename M1> static inline void inverse(const M1& a, M1& b) {
                typename M1::T d = det_t<2,structural_ids::normal>::det<typename M1::T>(a);
                assert(d != 0);
                typename M1::T reciprocal = 1. / d;
                b.set(0, 0) = reciprocal * a.get(1, 1);
                b.set(0, 1) = -reciprocal * a.get(0, 1);
                b.set(1, 0) = -reciprocal * a.get(1, 0);
                b.set(1, 1) = reciprocal * a.get(0, 0);
            }
        };
        template <> struct inverse_t<2,structural_ids::diagonal> {
            template <typename M1> static inline void inverse(const M1& a, M1& b) {
                b.set(0, 0) = static_cast<typename M1::T>(1) / a.get(0, 0);
                b.set(1, 1) = static_cast<typename M1::T>(1) / a.get(1, 1);
            }
        };
        template <> struct inverse_t<3,structural_ids::normal> {
            template <typename M1> static inline void inverse(const M1& a, M1& b) {
                typename M1::T d = det_t<3,structural_ids::normal>::det<typename M1::T>(a);
                assert(d != 0);
                typename M1::T reciprocal = 1. / d;
                b.set(0, 0) = reciprocal * (a.get(1, 1) * a.get(2, 2) - a.get(1, 2) * a.get(2, 1));
                b.set(0, 1) = reciprocal * (a.get(0, 2) * a.get(2, 1) - a.get(0, 1) * a.get(2, 2));
                b.set(0, 2) = reciprocal * (a.get(0, 1) * a.get(1, 2) - a.get(0, 2) * a.get(1, 1));
                b.set(1, 0) = reciprocal * (a.get(1, 2) * a.get(2, 0) - a.get(1, 0) * a.get(2, 2));
                b.set(1, 1) = reciprocal * (a.get(0, 0) * a.get(2, 2) - a.get(0, 2) * a.get(2, 0));
                b.set(1, 2) = reciprocal * (a.get(0, 2) * a.get(1, 0) - a.get(0, 0) * a.get(1, 2));
                b.set(2, 0) = reciprocal * (a.get(1, 0) * a.get(2, 1) - a.get(1, 1) * a.get(2, 0));
                b.set(2, 1) = reciprocal * (a.get(0, 1) * a.get(2, 0) - a.get(0, 0) * a.get(2, 1));
                b.set(2, 2) = reciprocal * (a.get(0, 0) * a.get(1, 1) - a.get(0, 1) * a.get(1, 0));
            }
        };
        template <> struct inverse_t<3,structural_ids::diagonal> {
            template <typename M1> static inline void inverse(const M1& a, M1& b) {
                b.set(0, 0) = static_cast<typename M1::T>(1) / a.get(0, 0);
                b.set(1, 1) = static_cast<typename M1::T>(1) / a.get(1, 1);
                b.set(2, 2) = static_cast<typename M1::T>(1) / a.get(2, 2);
            }
        };
        template <typename M1>
        inline void inverse(u8 R, structural_ids::en S, const M1& a, M1& b)
        {
            switch (R)
            {
                case 1:
                {
                    b.set(0) = 1. / a.get(0);
                    break;
                }
                case 2:
                {
                    if (structural_ids::normal == S)
                    {
                        typename M1::T d = det(R, S, a);
                        assert(d != 0);
                        typename M1::T reciprocal = 1. / d;
                        b.set(0, 0) = reciprocal * a.get(1, 1);
                        b.set(0, 1) = -reciprocal * a.get(0, 1);
                        b.set(1, 0) = -reciprocal * a.get(1, 0);
                        b.set(1, 1) = reciprocal * a.get(0, 0);
                    }
                    else if (structural_ids::diagonal == S)
                    {
                        b.set(0, 0) = static_cast<typename M1::T>(1) / a.get(0, 0);
                        b.set(1, 1) = static_cast<typename M1::T>(1) / a.get(1, 1);
                    }
                    break;
                }
                case 3:
                {
                    if (structural_ids::normal == S)
                    {
                        typename M1::T d = det(R, S, a);
                        assert(d != 0);
                        typename M1::T reciprocal = 1. / d;
                        b.set(0, 0) = reciprocal * (a.get(1, 1) * a.get(2, 2) - a.get(1, 2) * a.get(2, 1));
                        b.set(0, 1) = reciprocal * (a.get(0, 2) * a.get(2, 1) - a.get(0, 1) * a.get(2, 2));
                        b.set(0, 2) = reciprocal * (a.get(0, 1) * a.get(1, 2) - a.get(0, 2) * a.get(1, 1));
                        b.set(1, 0) = reciprocal * (a.get(1, 2) * a.get(2, 0) - a.get(1, 0) * a.get(2, 2));
                        b.set(1, 1) = reciprocal * (a.get(0, 0) * a.get(2, 2) - a.get(0, 2) * a.get(2, 0));
                        b.set(1, 2) = reciprocal * (a.get(0, 2) * a.get(1, 0) - a.get(0, 0) * a.get(1, 2));
                        b.set(2, 0) = reciprocal * (a.get(1, 0) * a.get(2, 1) - a.get(1, 1) * a.get(2, 0));
                        b.set(2, 1) = reciprocal * (a.get(0, 1) * a.get(2, 0) - a.get(0, 0) * a.get(2, 1));
                        b.set(2, 2) = reciprocal * (a.get(0, 0) * a.get(1, 1) - a.get(0, 1) * a.get(1, 0));
                    }
                    else if (structural_ids::diagonal == S)
                    {
                        b.set(0, 0) = static_cast<typename M1::T>(1) / a.get(0, 0);
                        b.set(1, 1) = static_cast<typename M1::T>(1) / a.get(1, 1);
                        b.set(2, 2) = static_cast<typename M1::T>(1) / a.get(2, 2);
                    }
                    break;
                }
                default:
                    assert(false); // unimplemented
            }
        }
    }
}