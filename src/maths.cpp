#include "maths.h"


// linear algebra
std::array<float, 3> cross(const std::array<float, 3> &a,
                           const std::array<float, 3> &b)
{
  return {
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]};
}

// analytic inverse for a 3x3 matrix
bool mat3x3_inverse(const Matrix<float, 3, 3>& A, Matrix<float, 3, 3>& invA)
{
  float det {A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])};
  if (std::fabs(det) < 1e-12f)
    return false;
  float invdet {1.0f / det};
  invA[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invdet;
  invA[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * invdet;
  invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invdet;
  invA[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * invdet;
  invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invdet;
  invA[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * invdet;
  invA[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invdet;
  invA[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * invdet;
  invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invdet;
  return true;
}

/*
 * Cholesky decomposition function
 * Finds given a matrix A, such that A = L * L_T
 * A - must be positive definite and symmetric
 */
void cholesky_7x7(const Matrix<float, 7, 7>& A, Matrix<float, 7, 7>& L_mat)
{
  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      L_mat[i][j] = 0;
    }
  }

  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j <= i; j++)
    {
      float s = 0;
      for (int k = 0; k < j; k++)
      {
        s += L_mat[i][k] * L_mat[j][k];
      }
      if (i == j)
      {
        float val = A[i][i] - s;
        if (val <= 0)
        {
          val = 1e-12f; // numerical safety
        }
        L_mat[i][j] = std::sqrt(val);
      }
      else
      {
        L_mat[i][j] = (1.0f / L_mat[j][j]) * (A[i][j] - s);
      }
    }
  }
}

// Hamilton product
/*
Quaternion quaternion_multiply(const Quaternion &a, const Quaternion &b)
{
  float newW = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z); // real part
  float newX = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y); // i
  float newY = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x); // j
  float newZ = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w); // k

  return Quaternion{newW, newX, newY, newZ};
}
*/


// quaternions
Quaternion integrate_omega(const Vector<float, 3>& omega, float dt)
{
  // quaternion approx for delta rotation: q = [cos(|theta|/2), u*sin(|theta|/2)]
  float wx {omega[0] * dt};
  float wy {omega[1] * dt};
  float wz {omega[2] * dt};
  float theta {std::sqrt(wx * wx + wy * wy + wz * wz)}; // total angle change
  Vector<float, 4> q;
  if (theta < 1e-9f)
  {
    q = {1.0f, 0.5f * wx, 0.5f * wy, 0.5f * wz};
  }
  else
  {
    float half {0.5f * theta};
    float s {std::sin(half) / theta};
    q[0] = std::cos(half);
    q[1] = wx * s;
    q[2] = wy * s;
    q[3] = wz * s;
  }

  return Quaternion{q[0], q[1], q[2], q[3]};
}

void normalize_quaternion(Quaternion &q)
{
  float length {static_cast<float>(std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z))};
  //return Quaternion{q.w / length, q.x / length, q.y / length, q.z / length};
  q = {q.w / length, q.x / length, q.y / length, q.z / length};
}

// Hamilton product
Quaternion operator*(const Quaternion& a, const Quaternion& b){
  float w {(a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z)}; // real part
  float x {(a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y)}; // i
  float y {(a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x)}; // j
  float z {(a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w)}; // k

  return Quaternion{w, x, y, z};
}

// Optimized solution
// (an alternative to $$\mathbf{z}^{(i)} = (q^{(i)})^* \otimes \mathbf{g} \otimes q^{(i)}$$)
// https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
// 2(u⋅v)u + (s2−u⋅u)v + 2s(u×v)
Vector<float, 3> rotate_vector_by_quaternion(const Vector<float, 3> &v, const Quaternion &q)
{
  Vector<float, 3> u = {q.x, q.y, q.z};
  float s = q.w;

  // $$2 * (u \cdot v) * u$$
  Vector<float, 3> term1 {scalar_multiplication(u, 2.0f * dot(u, v))};

  // $$s*s - (u \cdot u) * v$$
  Vector<float, 3> term2 {scalar_multiplication(v, (s * s - dot(u, u)))};

  // $$2 * s * (u \times v)$$
  Vector<float, 3> term3 {scalar_multiplication(cross(u, v), 2.0f * s)};

  Vector<float, 3> out;

  out[0] = term1[0] + term2[0] + term3[0];
  out[1] = term1[1] + term2[1] + term3[1];
  out[2] = term1[2] + term2[2] + term3[2];

  return out;
}