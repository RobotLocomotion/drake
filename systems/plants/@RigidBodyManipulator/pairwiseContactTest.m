function b = pairwiseContactTest(obj,body_ind1,body_ind2)

b = collisionmex(obj.mex_model_ptr.getData,1,body_ind1,body_ind2);
