#pragma once
#include <math.h>
#include <array>
#include <cstddef>
#include "const.h"


// linear algebra

template <typename T, std::size_t Row, std::size_t Col>
using Matrix = std::array<std::array<T, Col>, Row>;

template <typename T, std::size_t N>
using Vector = std::array<T, N>;

template <size_t N>
float dot(const Vector<float, N> &a, const Vector<float, N> &b)
{
    float sum = 0.0f;
    for (std::size_t i = 0; i < N; i++)
        sum += a[i] * b[i];
    return sum;
}

template <typename T, size_t N>
std::array<T, N> scalar_multiplication(const std::array<T, N> &v, T s)
{
    std::array<T, N> out{};
    for (size_t i = 0; i < N; i++)
    {
        out[i] = v[i] * s;
    }
    return out;
}

template <typename T, size_t N>
std::array<T, N> add(const std::array<T, N> &a, const std::array<T, N> &b)
{
    std::array<T, N> out{};
    for (size_t i = 0; i < N; i++)
    {
        out[i] = a[i] + b[i];
    }
    return out;
}

template<std::size_t Row, std::size_t Col>
void matrix_scalar_multiplication(Matrix<float, Row, Col>& A, const float scalar)
{
  for (int i = 0; i < Row; i++){
    for (int j = 0; j < Col; j++)
      A[i][j] *= scalar;
      }
}

template <typename T, size_t N>
Matrix<T, 1, N> matrix_scalar_multiplication(const Matrix<T, 1, N> &v, T s);

template <typename T, size_t N>
Matrix<T, 1, N> matrix_addition(const Matrix<T, 1, N> &a, const Matrix<T, 1, N> &b);

std::array<float, 3> cross(const std::array<float, 3> &a,
                           const std::array<float, 3> &b);

bool mat3x3_inverse(const Matrix<float, 3, 3>& A, Matrix<float, 3, 3>& invA);

void cholesky_7x7(const Matrix<float, 7, 7>& A, Matrix<float, 7, 7>& Lmat); // for 7x7 matrices

template <std::size_t m, std::size_t n, std::size_t p>
Matrix<float, m, p> matrix_multiplication(const Matrix<float, m, n>& A, const Matrix<float, n, p>& B){
    Matrix<float, m, p> AB;

    for (int i = 0; i < m; i++)
        for (int j = 0; j < p; j++)
        {
            AB[i][j] = 0;
            for (int k = 0; k < n; k++)
                AB[i][j] += A[i][k] * B[k][j];
        }
    return AB;
}

template <std::size_t Row, std::size_t Col>
Matrix<float, Row, Col> operator+(const Matrix<float, Row, Col>& A, const Matrix<float, Row, Col>& B){
    Matrix<float, Row, Col> A_plus_B;

    for(std::size_t i = 0; i < Row; i++){
        for(std::size_t j = 0; j < Col; j++){
            A_plus_B[i][j] = A[i][j] + B[i][j];
        }
    }

    return A_plus_B;
}

template <std::size_t Row, std::size_t Col>
Matrix<float, Row, Col> operator-(const Matrix<float, Row, Col>& A, const Matrix<float, Row, Col>& B){
    Matrix<float, Row, Col> A_minus_B;

    for(std::size_t i = 0; i < Row; i++){
        for(std::size_t j = 0; j < Col; j++){
            A_minus_B[i][j] = A[i][j] - B[i][j];
        }
    }

    return A_minus_B;
}


template <std::size_t m, std::size_t n, std::size_t p>
Matrix<float, m, p> operator*(const Matrix<float, m, n>& A, const Matrix<float, n, p>& B){
    Matrix<float, m, p> AB;

    for (int i = 0; i < m; i++){
        for (int j = 0; j < p; j++)
        {
            AB[i][j] = 0;
            for (int k = 0; k < n; k++)
                AB[i][j] += A[i][k] * B[k][j];
        }
    }
    return AB;
}

template<std::size_t Row, std::size_t Col>
Matrix<float, Col, Row> transpose_matrix(const Matrix<float, Row, Col>& A){
  Matrix<float, Col, Row> A_transposed;

  for(std::size_t i = 0; i < Row; i++){
    for(std::size_t j = 0; j < Col; j++){
      A_transposed[j][i] = A[i][j];
    }
  }

  return A_transposed;
}

template<std::size_t Row>
Vector<float, Row> operator-(const Vector<float, Row>& v1, const Vector<float, Row>& v2){
    Vector<float, Row> v1_minus_v2;
    for(std::size_t i = 0; i < Row; i++){
        v1_minus_v2[i] = v1[i] - v2[i];
    }

    return v1_minus_v2;
}


// matrix vector multiplication
template<std::size_t Row, std::size_t Col>
Vector<float, Row> operator*(const std::array<std::array<float, Col>, Row>& A, const std::array<float, Col>& v)
{
    Vector<float, Row> A_times_v;
        for (std::size_t i = 0; i < Row; i++)
        {
            A_times_v[i] = 0.0f;
            for (std::size_t j = 0; j < Col; j++)
            {
                A_times_v[i] += A[i][j] * v[j];
            }
        }
    return A_times_v;
}


template<std::size_t Row>
Vector<float, Row> operator*(const Vector<float, Row>& v, const float& s){
    Vector<float, Row> newVector;
    for (std::size_t i = 0; i < Row; i++)
    {
        newVector[i] = v[i] * s;
    }

    return newVector;
}


// Quaternion

struct Quaternion
{
    float w, x, y, z;

    constexpr Quaternion(float _w, float _x, float _y, float _z)
        : w(_w), x(_x), y(_y), z(_z){}

    constexpr Quaternion() : w(1), x(0), y(0), z(0){}
};

Quaternion operator*(const Quaternion& q, const Quaternion& b);

Quaternion quaternion_multiply(const Quaternion &a, const Quaternion &b);

void normalize_quaternion(Quaternion &q);

Vector<float, 3> rotate_vector_by_quaternion(const Vector<float, 3> &v, const Quaternion &q);

Quaternion integrate_omega(const Vector<float, 3> &omega, float dt);