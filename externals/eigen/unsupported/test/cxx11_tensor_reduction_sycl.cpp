// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2015
// Mehdi Goli    Codeplay Software Ltd.
// Ralph Potter  Codeplay Software Ltd.
// Luke Iwanski  Codeplay Software Ltd.
// Contact: <eigen@codeplay.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#define EIGEN_TEST_NO_LONGDOUBLE
#define EIGEN_TEST_NO_COMPLEX
#define EIGEN_TEST_FUNC cxx11_tensor_reduction_sycl
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#define EIGEN_USE_SYCL

#include "main.h"
#include <unsupported/Eigen/CXX11/Tensor>


template <typename DataType, int DataLayout>
static void test_full_reductions_sycl(const Eigen::SyclDevice&  sycl_device) {

  const int num_rows = 452;
  const int num_cols = 765;
  array<int, 2> tensorRange = {{num_rows, num_cols}};

  Tensor<DataType, 2, DataLayout> in(tensorRange);
  Tensor<DataType, 0, DataLayout> full_redux;
  Tensor<DataType, 0, DataLayout> full_redux_gpu;

  in.setRandom();

  full_redux = in.sum();

  DataType* gpu_in_data = static_cast<DataType*>(sycl_device.allocate(in.dimensions().TotalSize()*sizeof(DataType)));
  DataType* gpu_out_data =(DataType*)sycl_device.allocate(sizeof(DataType));

  TensorMap<Tensor<DataType, 2, DataLayout> >  in_gpu(gpu_in_data, tensorRange);
  TensorMap<Tensor<DataType, 0, DataLayout> >  out_gpu(gpu_out_data);

  sycl_device.memcpyHostToDevice(gpu_in_data, in.data(),(in.dimensions().TotalSize())*sizeof(DataType));
  out_gpu.device(sycl_device) = in_gpu.sum();
  sycl_device.memcpyDeviceToHost(full_redux_gpu.data(), gpu_out_data, sizeof(DataType));
  // Check that the CPU and GPU reductions return the same result.
  VERIFY_IS_APPROX(full_redux_gpu(), full_redux());

  sycl_device.deallocate(gpu_in_data);
  sycl_device.deallocate(gpu_out_data);
}
template <typename DataType, int DataLayout>
static void test_first_dim_reductions_sycl(const Eigen::SyclDevice& sycl_device) {

  int dim_x = 145;
  int dim_y = 1;
  int dim_z = 67;

  array<int, 3> tensorRange = {{dim_x, dim_y, dim_z}};
  Eigen::array<int, 1> red_axis;
  red_axis[0] = 0;
  array<int, 2> reduced_tensorRange = {{dim_y, dim_z}};

  Tensor<DataType, 3, DataLayout> in(tensorRange);
  Tensor<DataType, 2, DataLayout> redux(reduced_tensorRange);
  Tensor<DataType, 2, DataLayout> redux_gpu(reduced_tensorRange);

  in.setRandom();

  redux= in.sum(red_axis);

  DataType* gpu_in_data = static_cast<DataType*>(sycl_device.allocate(in.dimensions().TotalSize()*sizeof(DataType)));
  DataType* gpu_out_data = static_cast<DataType*>(sycl_device.allocate(redux_gpu.dimensions().TotalSize()*sizeof(DataType)));

  TensorMap<Tensor<DataType, 3, DataLayout> >  in_gpu(gpu_in_data, tensorRange);
  TensorMap<Tensor<DataType, 2, DataLayout> >  out_gpu(gpu_out_data, reduced_tensorRange);

  sycl_device.memcpyHostToDevice(gpu_in_data, in.data(),(in.dimensions().TotalSize())*sizeof(DataType));
  out_gpu.device(sycl_device) = in_gpu.sum(red_axis);
  sycl_device.memcpyDeviceToHost(redux_gpu.data(), gpu_out_data, redux_gpu.dimensions().TotalSize()*sizeof(DataType));

  // Check that the CPU and GPU reductions return the same result.
  for(int j=0; j<reduced_tensorRange[0]; j++ )
    for(int k=0; k<reduced_tensorRange[1]; k++ )
      VERIFY_IS_APPROX(redux_gpu(j,k), redux(j,k));

  sycl_device.deallocate(gpu_in_data);
  sycl_device.deallocate(gpu_out_data);
}

template <typename DataType, int DataLayout>
static void test_last_dim_reductions_sycl(const Eigen::SyclDevice &sycl_device) {

  int dim_x = 567;
  int dim_y = 1;
  int dim_z = 47;

  array<int, 3> tensorRange = {{dim_x, dim_y, dim_z}};
  Eigen::array<int, 1> red_axis;
  red_axis[0] = 2;
  array<int, 2> reduced_tensorRange = {{dim_x, dim_y}};

  Tensor<DataType, 3, DataLayout> in(tensorRange);
  Tensor<DataType, 2, DataLayout> redux(reduced_tensorRange);
  Tensor<DataType, 2, DataLayout> redux_gpu(reduced_tensorRange);

  in.setRandom();

  redux= in.sum(red_axis);

  DataType* gpu_in_data = static_cast<DataType*>(sycl_device.allocate(in.dimensions().TotalSize()*sizeof(DataType)));
  DataType* gpu_out_data = static_cast<DataType*>(sycl_device.allocate(redux_gpu.dimensions().TotalSize()*sizeof(DataType)));

  TensorMap<Tensor<DataType, 3, DataLayout> >  in_gpu(gpu_in_data, tensorRange);
  TensorMap<Tensor<DataType, 2, DataLayout> >  out_gpu(gpu_out_data, reduced_tensorRange);

  sycl_device.memcpyHostToDevice(gpu_in_data, in.data(),(in.dimensions().TotalSize())*sizeof(DataType));
  out_gpu.device(sycl_device) = in_gpu.sum(red_axis);
  sycl_device.memcpyDeviceToHost(redux_gpu.data(), gpu_out_data, redux_gpu.dimensions().TotalSize()*sizeof(DataType));
  // Check that the CPU and GPU reductions return the same result.
  for(int j=0; j<reduced_tensorRange[0]; j++ )
    for(int k=0; k<reduced_tensorRange[1]; k++ )
      VERIFY_IS_APPROX(redux_gpu(j,k), redux(j,k));

  sycl_device.deallocate(gpu_in_data);
  sycl_device.deallocate(gpu_out_data);

}
template<typename DataType> void sycl_reduction_test_per_device(const cl::sycl::device& d){
  std::cout << "Running on " << d.template get_info<cl::sycl::info::device::name>() << std::endl;
  QueueInterface queueInterface(d);
  auto sycl_device = Eigen::SyclDevice(&queueInterface);

  test_full_reductions_sycl<DataType, RowMajor>(sycl_device);
  test_first_dim_reductions_sycl<DataType, RowMajor>(sycl_device);
  test_last_dim_reductions_sycl<DataType, RowMajor>(sycl_device);
  test_full_reductions_sycl<DataType, ColMajor>(sycl_device);
  test_first_dim_reductions_sycl<DataType, ColMajor>(sycl_device);
  test_last_dim_reductions_sycl<DataType, ColMajor>(sycl_device);
}
void test_cxx11_tensor_reduction_sycl() {
  for (const auto& device :Eigen::get_sycl_supported_devices()) {
    CALL_SUBTEST(sycl_reduction_test_per_device<float>(device));
  }
}
