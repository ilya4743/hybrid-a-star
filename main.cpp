#define BOOST_TEST_MODULE boost_test_macro2
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE( test_op_precedence )
{
  int a = 13, b = 2, c = 12;
  // left term of == is expanded in the logs
  BOOST_TEST(a % b == c);
  // right term of == is not expanded in the logs
  BOOST_TEST(a == c % b);
}

BOOST_AUTO_TEST_CASE( TEST_ECB )
{
  int a = 1;
  for(int i=0; i<10; i++)
  {
    BOOST_CHECK_EQUAL(a,0);
  }
  BOOST_TEST(!a);
  BOOST_TEST(--a);
}