#pragma once

template <typename T>
struct ParticleData {
  std::vector<Vector3<T>> x;
  std::vector<Vector3<T>> v;
  std::vector<Matrix3<T>> F;
  std::vector<Matrix3<T>> C;
  std::vector<Matrix3<T>> P;
};