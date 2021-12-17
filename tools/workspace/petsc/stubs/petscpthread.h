#pragma once

#include <pthread.h>

#include <petscsys.h>

typedef pthread_mutex_t PetscSpinlock;

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockCreate(PetscSpinlock* pthread_mutex) {
  pthread_mutex_init(pthread_mutex, NULL);
  return 0;
}

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockLock(PetscSpinlock* pthread_mutex) {
  pthread_mutex_lock(pthread_mutex);
  return 0;
}

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockUnlock(PetscSpinlock* pthread_mutex) {
  pthread_mutex_unlock(pthread_mutex);
  return 0;
}

PETSC_STATIC_INLINE PetscErrorCode
PetscSpinlockDestroy(PetscSpinlock* pthread_mutex) {
  pthread_mutex_destroy(pthread_mutex);
  return 0;
}
