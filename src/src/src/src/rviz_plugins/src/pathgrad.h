#ifndef RVIZ_PATH_GRAD_H
#define RVIZ_PATH_GRAD_H

#include <nav_msgs/Path.h>
#include "rviz/message_filter_display.h"
#include "ColourGradient.h"
namespace Ogre
{
class ManualObject;
}

namespace rviz
{
class IntProperty;
class FloatProperty;
class ColorProperty;
class RosTopicProperty;
class EnumProperty;
class BillboardLine;
class PathGradDisplay:public MessageFilterDisplay<nav_msgs::Path>{
    Q_OBJECT
public:
    PathGradDisplay();
    virtual ~PathGradDisplay();
    virtual void reset();

protected:
    virtual void onInitialize();
    void processMessage( const nav_msgs::Path::ConstPtr& msg );
    
protected Q_SLOTS:
    void updateBufferLength();
    void updateStyle();
    void updateLineWidth();

protected:
    void destroyObjects();
    
    // 绘图相关
    std::vector<Ogre::ManualObject*> manual_objects_;
    std::vector<rviz::BillboardLine*> billboard_lines_;

    boost::mutex  mutex_;

    enum LineStyle {
        LINES,
        BILLBOARDS
    };
   
    
    EnumProperty* style_property_;
    ColorProperty* min_color_property_;
    ColorProperty* max_color_property_;
    FloatProperty* alpha_property_;
    FloatProperty* line_width_property_;
    IntProperty* buffer_length_property_;

};

} // end namespace rviz


#endif