#pragma once
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>

#define PCTypes (pcl::PointXYZ)(pcl::PointXYZI)

#define AddFixedType(r, name, elem)     \
((elem)(name))                          \

#define PCTypesWithFixedType(type)                    \
BOOST_PP_SEQ_FOR_EACH(AddFixedType, type, PCTypes)    \

#define PCTemplateHelper(r, name, elem)  \
template class name<elem>;               \

#define PCTemplateInstantiateExplicitly(class_name)            \
BOOST_PP_SEQ_FOR_EACH(PCTemplateHelper, class_name, PCTypes)   \


#define PCTemplateHelperSeq(r, name, elem)          \
template class name<BOOST_PP_SEQ_ENUM(elem)>;       \


#define PCTemplateInstantiateExplicitlyWithFixedType(class_name, type)                  \
BOOST_PP_SEQ_FOR_EACH(PCTemplateHelperSeq, class_name, PCTypesWithFixedType(type))      \


