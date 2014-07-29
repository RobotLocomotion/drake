#ifndef __DrakeCollisionBulletElement_H__
#define __DrakeCollisionBulletElement_H__

namespace DrakeCollision
{
  class BulletElement 
  {
    friend class BulletModel;

    public:
      BulletElement(const Eigen::Matrix4d& T_elem_to_link, Shape shape, 
                    const std::vector<double>& params);

      void updateWorldTransform(const Eigen::Matrix4d& T_link_to_world);

      const Eigen::Matrix4d& getWorldTransform() const; 

      const Eigen::Matrix4d& getLinkTransform() const; 

      const Shape& getShape() const;

    protected:
      virtual void setWorldTransform(const Eigen::Matrix4d& T_elem_to_world);

      Eigen::Matrix4d T_elem_to_link;
      Eigen::Matrix4d T_elem_to_world;
      Shape shape;

      std::shared_ptr<btCollisionObject> bt_obj;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
