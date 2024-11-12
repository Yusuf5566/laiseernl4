/***
 *  @brief      Project
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-05-16
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_REFLECT_H
#define LAISEERNL4_REFLECT_H

#include <iostream>
#include <map>
#include <string>
#include <ros/ros.h>

template <typename ClassName>
class ClassRegister
{
public:
    typedef ClassName* (*Constructor)();

private:
    typedef std::map<std::string, Constructor> ClassMap;
    ClassMap constructor_map_;

public:
    void AddConstructor(const std::string class_name, Constructor constructor)
    {
        typename ClassMap::iterator it = constructor_map_.find(class_name);
        if (it != constructor_map_.end())
        {
            std::cout << "error!";
            return;
        }
        constructor_map_[class_name] = constructor;
    }

    ClassName* CreateObject(const std::string class_name) const
    {
        typename ClassMap::const_iterator it = constructor_map_.find(class_name);
        if (it == constructor_map_.end())
        {
            ROS_ERROR_STREAM("ClassName " << class_name << " ot found");
            return nullptr;
        }
        ROS_INFO_STREAM("ClassName " << class_name << " new ok");
        return (*(it->second))();
    }
};

template <typename ClassName>
ClassRegister<ClassName>& GetRegister()
{
    static ClassRegister<ClassName> class_register;
    return class_register;
}

template <typename BaseClassName, typename SubClassName>
BaseClassName* NewObject()
{
    return new SubClassName();
}

template <typename BaseClassName>
class ClassRegisterHelper
{
public:
    ClassRegisterHelper(const std::string sub_class_name,
        typename ClassRegister<BaseClassName>::Constructor constructor)
    {
        GetRegister<BaseClassName>().AddConstructor(sub_class_name, constructor);
    }
    ~ClassRegisterHelper() = default;
};

#define RegisterClass(base_class_name, sub_class_name)                                            \
    static ClassRegisterHelper<base_class_name> sub_class_name##_register_helper(#sub_class_name, \
        NewObject<base_class_name, sub_class_name>);

#define CreateObject(base_class_name, sub_class_name_as_string) \
    GetRegister<base_class_name>().CreateObject(sub_class_name_as_string)

#endif  // LAISEERNL4_REFLECT_H
