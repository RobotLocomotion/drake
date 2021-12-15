#pragma once

#include <assert.h>
#include <pthread.h>

#include <petscsys.h>

typedef pthread_mutex_t PetscSpinlock;

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockCreate(PetscSpinlock* pthread_mutex) {
  const int result = pthread_mutex_init(pthread_mutex, NULL);
  // Assert that mutex init was successful.
  assert(result == 0);
  return 0;
}

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockLock(PetscSpinlock* pthread_mutex) {
  const int result = pthread_mutex_lock(pthread_mutex);
  // Assert that mutex lock was successful.
  assert(result == 0);
  return 0;
}

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockUnlock(PetscSpinlock* pthread_mutex) {
  const int result = pthread_mutex_unlock(pthread_mutex);
  // Assert that mutex unlock was successful.
  assert(result == 0);
  return 0;
}

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockDestroy(PetscSpinlock* pthread_mutex) {
  const int result = pthread_mutex_destroy(pthread_mutex);
  // Assert that mutex destroy was successful.
  assert(result == 0);
  return 0;
}
