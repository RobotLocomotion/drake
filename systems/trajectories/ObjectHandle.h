/* class ObjectHandle: a handle class for mex-functions
 *
 * Notes:
 * 1. If passing an object to the first (pointer) constructor, be sure that
 * the object was allocated on the heap as the ObjectHandle takes ownership
 * of the objects and will eventually delete it. Be sure to not delete any aliases
 * to the passed object.
 * 2. The second (reference) constructor permits static objects to be passed via 
 * ObjectHandles by ensuring the ObjectHandle does not take ownership of
 * the object. 
 *
 * Tim Bailey 2004. Adapted from a design by Mike Stevens.
 */

#ifndef OBJECT_HANDLE_T_H_
#define OBJECT_HANDLE_T_H_

#include <mex.h>
#include <typeinfo>
#include <list>

template<typename T> class Collector;

template <typename T>
class ObjectHandle {
public:
	// Constructor for free-store allocated objects.
	// Handle takes ownership, and will delete object when it is destroyed.
	ObjectHandle(T*& ptr) : type(&typeid(T)), owns(true), t(ptr) { 
		signature = this; 
		Collector<T>::register_handle(this);
		ptr = 0;
	}

	// Constructor for non-owned objects.
	// Object may be heap or statically allocated; the handle does NOT
	// take ownership, and the client is responsible for deleting it.
	ObjectHandle(T& obj) : type(&typeid(T)), owns(false), t(&obj) { 
		signature= this; 
	} 

	~ObjectHandle() { 
		if (owns) delete t; // destroy object
		signature= 0; // destroy signature
	} 

	// Convert ObjectHandle<T> to a mxArray handle (to pass back from mex-function).
	mxArray* to_mex_handle(); 

	// Convert mxArray (passed to mex-function) to an ObjectHandle<T>.
	static ObjectHandle* from_mex_handle( const mxArray* ma );

	// Get the actual object contained by handle
	T& get_object() const { return *t; }

private:
	ObjectHandle* signature; // use 'this' as a unique object signature 
	const std::type_info* type; // type checking information
	bool owns; // marks whether handle owns pointed-to object
	T *t; // object pointer

	friend class Collector<T>; // allow Collector access to signature
};

// --------------------------------------------------------- 
// ------------------ Helper functions ---------------------
// --------------------------------------------------------- 
// These functions remove the need to deal with ObjectHandle<T>
// class directly for most common operations.

template <typename T>
mxArray *create_handle(T* t)
// Create mex handle to object t (where t is heap allocated). 
// Client no longer owns t, and so must not delete it.
{
	ObjectHandle<T>* handle= new ObjectHandle<T>(t);
	return handle->to_mex_handle();
}

template <typename T>
T& get_object(const mxArray *mxh)
// Obtain object represented by handle.
{
	ObjectHandle<T>* handle= ObjectHandle<T>::from_mex_handle(mxh);
	return handle->get_object();
}

template <typename T>
void destroy_object(const mxArray *mxh)
// If deleting object, rather than leaving it to garbage collection,
// must delete it via the handle; do not delete T* directly.
{
	ObjectHandle<T>* handle= ObjectHandle<T>::from_mex_handle(mxh);
	delete handle;
}

// --------------------------------------------------------- 
// ------------------ Garbage Collection -------------------
// --------------------------------------------------------- 

// Garbage collection singleton (one collector object for each type T).
// Ensures that registered handles are deleted when the dll is released (they
// may also be deleted previously without problem).
//    The Collector provides protection against resource leaks in the case
// where 'clear all' is called in MatLab. (This is because MatLab will call
// the destructors of statically allocated objects but not free-store allocated
// objects.)
template <typename T>
class Collector {
	std::list<ObjectHandle<T>*> objlist;
public:
	~Collector() {
		typename std::list<ObjectHandle<T>*>::iterator i;
		typename std::list<ObjectHandle<T>*>::iterator end= objlist.end();
		for (i= objlist.begin(); i!=end; ++i) {
			if ((*i)->signature == *i) // check for valid signature
				delete *i;
		}
	}

	static void register_handle (ObjectHandle<T>* obj) {
		static Collector singleton;
		singleton.objlist.push_back(obj);
	}

private: // prevent construction
	Collector() {}
	Collector(const Collector&);
};

// --------------------------------------------------------- 
// ---------- Implementation of member functions ----------- 
// --------------------------------------------------------- 

// Import a handle from MatLab as a mxArray of UINT32. Check that
// it is actually a pointer to an ObjectHandle<T>.
template <typename T>
ObjectHandle<T>* ObjectHandle<T>::from_mex_handle(const mxArray* handle) 
{
	if (mxGetClassID(handle) != mxUINT32_CLASS 
		|| mxIsComplex(handle) || mxGetM(handle)!=1 || mxGetN(handle)!=1)
		mexErrMsgTxt("Parameter is not an ObjectHandle type.");

	// We *assume* we can store ObjectHandle<T> pointer in the mxUINT32 of handle
	ObjectHandle* obj = *reinterpret_cast<ObjectHandle<T>**>(mxGetPr(handle));

	if (!obj) // gross check to see we don't have an invalid pointer
		mexErrMsgTxt("Parameter is NULL. It does not represent an ObjectHandle object.");
// TODO: change this for max-min check for pointer values

	if (obj->signature != obj) // check memory has correct signature
		mexErrMsgTxt("Parameter does not represent an ObjectHandle object.");

	if (*(obj->type) != typeid(T)) { // check type 
		mexPrintf("Given: <%s>, Required: <%s>.\n", obj->type->name(), typeid(T).name());
		mexErrMsgTxt("Given ObjectHandle does not represent the correct type.");
	}

	return obj;
}

// Create a numeric array as handle for an ObjectHandle.
// We ASSUME we can store object pointer in the mxUINT32 element of mxArray.
template <typename T>
mxArray* ObjectHandle<T>::to_mex_handle() 
{
	mxArray* handle  = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
	*reinterpret_cast<ObjectHandle<T>**>(mxGetPr(handle)) = this;
	return handle;
}

#endif
