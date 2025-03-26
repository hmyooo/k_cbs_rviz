#include "pathgrad.h"

#include <boost/bind.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>

#include <ros/ros.h>
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/line.h"
#include "rviz/ogre_helpers/billboard_line.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h" 
namespace rviz
{

PathGradDisplay::PathGradDisplay(){

  style_property_ = new EnumProperty( "Line Style", "Lines",
                                       "The rendering operation to use to draw the grid lines.",
                                       this, SLOT( updateStyle() ));
  style_property_->addOption( "Lines", LINES );
  style_property_->addOption( "Billboards", BILLBOARDS );
  line_width_property_ = new FloatProperty( "Line Width", 0.03,
                                             "The width, in meters, of each path line."
                                             "Only works with the 'Billboards' style.",
                                             this, SLOT( updateLineWidth() ), this );
  line_width_property_->setMin( 0.001 );
  line_width_property_->hide();
  min_color_property_ = new ColorProperty("min Color", Qt::white, "Color to draw the range.", this);
  max_color_property_ = new ColorProperty("max Color", Qt::black, "Color to draw the range.", this);
  alpha_property_ = new FloatProperty( "Alpha", 1.0,
                    "Amount of transparency to apply to the path.", this );
  buffer_length_property_ = new IntProperty( "Buffer Length", 1,
                                        "Number of paths to display.",
                                        this, SLOT( updateBufferLength() ));
  buffer_length_property_->setMin( 1 );

}

PathGradDisplay::~PathGradDisplay(){
  destroyObjects();
}
void PathGradDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
}

void PathGradDisplay::reset(){
  MFDClass::reset();
  updateBufferLength();
}

void PathGradDisplay::updateStyle(){
  LineStyle style = (LineStyle) style_property_->getOptionInt(); 
  switch( style ){
    case LINES:
    default:
      line_width_property_->hide();
      break;
    case BILLBOARDS:
      line_width_property_->show();
      break;
  }
  updateBufferLength();
}

void PathGradDisplay::updateLineWidth(){
  LineStyle style = (LineStyle) style_property_->getOptionInt();
  float line_width = line_width_property_->getFloat();
  if(style == BILLBOARDS) {
    for( size_t i = 0; i < billboard_lines_.size(); i++ ){
      rviz::BillboardLine* billboard_line = billboard_lines_[ i ];
      if( billboard_line ) billboard_line->setLineWidth( line_width );
    }
  }
  context_->queueRender();
}

void PathGradDisplay::updateBufferLength(){
   // Delete old path objects
   destroyObjects();

   // Read options
   int buffer_length = buffer_length_property_->getInt();
   LineStyle style = (LineStyle) style_property_->getOptionInt();
 
   // Create new path objects
   switch(style){
   case LINES: // simple lines with fixed width of 1px
     manual_objects_.resize( buffer_length );
     for( size_t i = 0; i < manual_objects_.size(); i++ )
     {
       Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
       manual_object->setDynamic( true );
       scene_node_->attachObject( manual_object ); 
       manual_objects_[ i ] = manual_object;
     }
     break;
 
   case BILLBOARDS: // billboards with configurable width
     billboard_lines_.resize( buffer_length );
     for( size_t i = 0; i < billboard_lines_.size(); i++ )
     {
       rviz::BillboardLine* billboard_line = new rviz::BillboardLine(scene_manager_, scene_node_);
       billboard_lines_[ i ] = billboard_line;
     }
     break;
   }

 
}

void PathGradDisplay::destroyObjects(){
  // Destroy all simple lines, if any
  for( size_t i = 0; i < manual_objects_.size(); i++ ){
    Ogre::ManualObject*& manual_object = manual_objects_[ i ];
    if( manual_object ){
      manual_object->clear();
      scene_manager_->destroyManualObject( manual_object );
      manual_object = NULL; // ensure it doesn't get destroyed again
    }
  }
  // Destroy all billboards, if any
  for( size_t i = 0; i < billboard_lines_.size(); i++ ){
    rviz::BillboardLine*& billboard_line = billboard_lines_[ i ];
    if( billboard_line ){
      delete billboard_line; // also destroys the corresponding scene node
      billboard_line = NULL; // ensure it doesn't get destroyed again
    }
  }
}

bool validateFloats( const nav_msgs::Path& msg ){
    bool valid = true;
    valid = valid && validateFloats( msg.poses );
    return valid;
}

float* range_velocity(const nav_msgs::Path& msg ){
  float* v_range = new float[2];
  v_range[0] = 9999;
  v_range[1] = -9999;
  for( uint32_t i=0; i < msg.poses.size(); ++i){
    // 速度修改处1
    const float velocity = msg.poses[i].pose.position.z;
    if(velocity < v_range[0]) v_range[0] = velocity;
    if(velocity > v_range[1]) v_range[1] = velocity;
  }
  return v_range;
} 


// ***********************************************************************************************************************************

void PathGradDisplay::processMessage( const nav_msgs::Path::ConstPtr& msg ){
  // Calculate index of oldest element in cyclic buffer
  size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();

  LineStyle style = (LineStyle) style_property_->getOptionInt();
  Ogre::ManualObject* manual_object = NULL;
  rviz::BillboardLine* billboard_line = NULL;

  // Delete oldest element
  switch(style){
    case LINES:
      manual_object = manual_objects_[ bufferIndex ];
      manual_object->clear();
      break;

    case BILLBOARDS:
      billboard_line = billboard_lines_[ bufferIndex ];
      billboard_line->clear();
      break;
  }

  // Check if path contains invalid coordinate values
  if( !validateFloats( *msg )){
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  float *v_range = range_velocity( *msg );
  float v_r = std::abs(v_range[1] - v_range[0]);
  uint32_t num_points = msg->poses.size();
  float line_width = line_width_property_->getFloat();
  Ogre::ColourValue min_color = min_color_property_->getOgreColor();
  Ogre::ColourValue max_color = max_color_property_->getOgreColor();
  ColourGradient gradcolor = ColourGradient(min_color,max_color);
  gradcolor.AddColourFrame(0.0, min_color);
  gradcolor.AddColourFrame(1.0,max_color);
  Ogre::ColourValue color;
  switch(style)  {
  case LINES:
    
    manual_object->estimateVertexCount( num_points );
    manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
    for( uint32_t i=0; i < num_points; ++i){
      const geometry_msgs::Point& pos = msg->poses[ i ].pose.position;
      // 速度修改处2
      const float v_d =  msg->poses[ i ].pose.position.z;
      // const float v_d =  msg->poses[ i ].pose.orientation.x;
      color = gradcolor.GetColour(std::abs(v_d - v_range[0])/v_r);
      color.a = alpha_property_->getFloat(); 
      // 速度修改处2
      // manual_object->position( pos.x, pos.y, pos.z);
      manual_object->position( pos.x, pos.y, 0);
      manual_object->colour( color );
    }

    manual_object->end();
    break;

  case BILLBOARDS:
    
    billboard_line->setNumLines( 1 );
    billboard_line->setMaxPointsPerLine( num_points );
    billboard_line->setLineWidth( line_width );
    for( uint32_t i=0; i < num_points; ++i){
      const geometry_msgs::Point& pos = msg->poses[ i ].pose.position;
      // 速度修改处2
      const float v_d =  msg->poses[ i ].pose.position.z;
      // const float v_d =  msg->poses[ i ].pose.orientation.x;
      color = gradcolor.GetColour(std::abs(v_d - v_range[0])/v_r);
      color.a = alpha_property_->getFloat();
      // 速度修改处2
      // Ogre::Vector3 xpos =  Ogre::Vector3( pos.x, pos.y, pos.z);
      Ogre::Vector3 xpos =  Ogre::Vector3( pos.x, pos.y, 0);
      billboard_line->addPoint( xpos, color );
    }

    break;
  }


  context_->queueRender();

}


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::PathGradDisplay, rviz::Display)
