#pragma once
#include <boost/preprocessor/seq/for_each.hpp>

#define PCTypes (pcl::PointXYZ)(pcl::PointXYZI)

#define PCRTemplateHelper(r, name, elem) \
template class name<elem>;               \

#define PCRTemplateInstantiateExplicitly(class_name)            \
BOOST_PP_SEQ_FOR_EACH(PCRTemplateHelper, class_name, PCTypes)   \

