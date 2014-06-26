#define BOOST_TEST_MODULE Body test
#include <boost/test/unit_test.hpp>

#include <iostream>

#include "DrakeCollision.h"
#include "BulletModel.h"

using namespace DrakeCollision;
using namespace std;

BOOST_AUTO_TEST_CASE(constructor_test)
{
  Body body1;
  BOOST_CHECK_EQUAL(body1.getBodyIdx(), -1);
  BOOST_CHECK_EQUAL(body1.getParentIdx(), -1);
  BOOST_CHECK_EQUAL(body1.getGroup(), DEFAULT_GROUP);
  BOOST_CHECK_EQUAL(body1.getMask(), ALL_MASK);
}

BOOST_AUTO_TEST_CASE(setGroup_test)
{
  Body body1;
  bitmask group; group.set(7);
  body1.setGroup(group);
  BOOST_CHECK_EQUAL(body1.getGroup(), group);
}

BOOST_AUTO_TEST_CASE(addToGroup_test)
{
  Body body1;
  bitmask group; group.set(7);
  body1.addToGroup(group);
  BOOST_CHECK( (body1.getGroup() & group) != 0 );
}

BOOST_AUTO_TEST_CASE(ignoreGroup_test)
{
  Body body1;
  bitmask group; group.set(7);
  body1.ignoreGroup(group);
  BOOST_CHECK( (body1.getMask() & group) == 0 );
}

BOOST_AUTO_TEST_CASE(collidesWith_test)
{
  Body body1;
  Body body2;
  body1.setBodyIdx(0);
  body2.setBodyIdx(1);
  BOOST_REQUIRE_MESSAGE( body1.collidesWith(body2),
                      "Default constructed objects should collide");
  body1.ignoreGroup(body2.getGroup());
  BOOST_CHECK_MESSAGE( !body1.collidesWith(body2),
                      "Bodies should not collide with bodies in ignored groups");
  BOOST_CHECK_EQUAL( body2.collidesWith(body1), body1.collidesWith(body2) );
}

BOOST_AUTO_TEST_CASE(collideWithGroup_test)
{
  Body body1;
  Body body2;
  body1.setBodyIdx(0);
  body2.setBodyIdx(1);
  body1.ignoreGroup(body2.getGroup());
  body1.collideWithGroup(body2.getGroup());
  BOOST_CHECK_MESSAGE( body1.collidesWith(body2), 
                      "Body 1 should collide with body2");
}
