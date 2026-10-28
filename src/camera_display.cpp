/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rviz_common/bit_allocator.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_common/properties/display_group_visibility_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/validate_floats.hpp>
#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderTexture.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <string>

#include "rviz_camera_stream/camera_display.h"

namespace video_export
{
class VideoPublisher
{
private:
  // ROS2: rviz node used in place of the ROS1 global node handle; nh_ keeps the NodeHandle namespace
  rclcpp::Node::SharedPtr node_;
  std::string nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher pub_;
  uint image_id_;
public:
  sensor_msgs::msg::CameraInfo camera_info_;
  explicit VideoPublisher(const rclcpp::Node::SharedPtr& node) :
    node_(node),
    it_(node_),
    image_id_(0)
  {
  }

  std::string get_topic()
  {
    return pub_.getTopic();
  }

  bool is_active()
  {
    return !pub_.getTopic().empty();
  }

  void setNodehandle(const std::string& nh)
  {
    shutdown();
    nh_ = nh;
  }

  void shutdown()
  {
    if (pub_.getTopic() != "")
    {
      pub_.shutdown();
    }
  }

  void advertise(std::string topic, const rclcpp::QoS& qos)
  {
    // ROS1 resolved a relative topic in the NodeHandle namespace
    if (!nh_.empty() && !topic.empty() && topic[0] != '/')
    {
      topic = nh_ + "/" + topic;
    }
    // ROS2: same as it_.advertiseCamera(topic, 1) with the default QoS
    pub_ = image_transport::create_camera_publisher(node_.get(), topic, qos.get_rmw_qos_profile());
  }

  // bool publishFrame(Ogre::RenderWindow * render_object, const std::string frame_id)
  bool publishFrame(Ogre::RenderTexture * render_object, const std::string frame_id, int encoding_option)
  {
    if (pub_.getTopic() == "")
    {
      return false;
    }
    if (frame_id == "")
    {
      return false;
    }
    // RenderTarget::writeContentsToFile() used as example
    // TODO(lucasw) make things const that can be
    int height = render_object->getHeight();
    int width = render_object->getWidth();
    // the suggested pixel format is most efficient, but other ones
    // can be used.
    sensor_msgs::msg::Image image;
    Ogre::PixelFormat pf = Ogre::PF_BYTE_RGB;
    switch (encoding_option)
    {
      case 0:
        pf = Ogre::PF_BYTE_RGB;
        image.encoding = sensor_msgs::image_encodings::RGB8;
        break;
      case 1:
        pf = Ogre::PF_BYTE_RGBA;
        image.encoding = sensor_msgs::image_encodings::RGBA8;
        break;
      case 2:
        pf = Ogre::PF_BYTE_BGR;
        image.encoding = sensor_msgs::image_encodings::BGR8;
        break;
      case 3:
        pf = Ogre::PF_BYTE_BGRA;
        image.encoding = sensor_msgs::image_encodings::BGRA8;
        break;
      case 4:
        pf = Ogre::PF_L8;
        image.encoding = sensor_msgs::image_encodings::MONO8;
        break;
      case 5:
        pf = Ogre::PF_L16;
        image.encoding = sensor_msgs::image_encodings::MONO16;
        break;
      default:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid image encoding value specified");
        return false;
    }

    uint pixelsize = Ogre::PixelUtil::getNumElemBytes(pf);
    uint datasize = width * height * pixelsize;

    // 1.05 multiplier is to avoid crash when the window is resized.
    // There should be a better solution.
    uchar *data = OGRE_ALLOC_T(uchar, static_cast<int>(datasize * 1.05), Ogre::MEMCATEGORY_RENDERSYS);
    Ogre::PixelBox pb(width, height, 1, pf, data);
    render_object->copyContentsToMemory(Ogre::Box(0, 0, width, height), pb, Ogre::RenderTarget::FB_AUTO);


    image.header.stamp = node_->now();
    image_id_++;  // ROS2 std_msgs/Header has no seq field
    image.header.frame_id = frame_id;
    image.height = height;
    image.width = width;
    image.step = pixelsize * width;
    image.is_bigendian = (OGRE_ENDIAN == OGRE_ENDIAN_BIG);
    image.data.resize(datasize);
    memcpy(&image.data[0], data, datasize);
    camera_info_.header = image.header;
    pub_.publish(image, camera_info_);

    OGRE_FREE(data, Ogre::MEMCATEGORY_RENDERSYS);
    return true;
  }
};
}  // namespace video_export


namespace rviz
{

// ROS2: rviz classes moved to the rviz_common namespaces
using rviz_common::FrameManagerIface;
using rviz_common::loadPixmap;
using rviz_common::validateFloats;
using rviz_common::properties::StatusProperty;

const QString CameraPub::BACKGROUND("background");
const QString CameraPub::OVERLAY("overlay");
const QString CameraPub::BOTH("background and overlay");

bool validateFloats(const sensor_msgs::msg::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.d);
  valid = valid && validateFloats(msg.k);
  valid = valid && validateFloats(msg.r);
  valid = valid && validateFloats(msg.p);
  return valid;
}

CameraPub::CameraPub()
  : Display()
  , camera_trigger_name_("camera_trigger")
  , nh_()
{
  topic_property_ = new RosTopicProperty("Image Topic", "",
      QString::fromStdString(rosidl_generator_traits::name<sensor_msgs::msg::Image>()),
      "sensor_msgs::Image topic to publish to.", this, SLOT(updateTopic()));

  namespace_property_ = new StringProperty("Display namespace", "",
      "Namespace for this display.", this, SLOT(updateDisplayNamespace()));

  camera_info_property_ = new RosTopicProperty("Camera Info Topic", "",
      QString::fromStdString(rosidl_generator_traits::name<sensor_msgs::msg::CameraInfo>()),
      "sensor_msgs::CameraInfo topic to subscribe to.", this, SLOT(updateTopic()));

  // ROS2: QoS settings under the topic properties, as in the rviz2 topic displays
  topic_qos_property_ = new QosProfileProperty(topic_property_, topic_qos_profile_);
  camera_info_qos_property_ = new QosProfileProperty(camera_info_property_, camera_info_qos_profile_);

  queue_size_property_ = new IntProperty( "Queue Size", 2,
      "Advanced: set the size of the incoming message queue.  Increasing this "
      "is useful if your incoming TF data is delayed significantly from your"
      " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT(updateQueueSize()));
  queue_size_property_->setMin(1);

  frame_rate_property_ = new FloatProperty("Frame Rate", -1,
      "Sets target frame rate. Set to < 0 for maximum speed, set to 0 to stop, you can "
      "trigger single images with the /rviz_camera_trigger service.",
                                           this, SLOT(updateFrameRate()));
  frame_rate_property_->setMin(-1);

  background_color_property_ = new ColorProperty("Background Color", Qt::black,
      "Sets background color, values from 0.0 to 1.0.",
                                           this, SLOT(updateBackgroundColor()));

  image_encoding_property_ = new EnumProperty("Image Encoding", "rgb8",
      "Sets the image encoding", this, SLOT(updateImageEncoding()));
  image_encoding_property_->addOption("rgb8", 0);
  image_encoding_property_->addOption("rgba8", 1);
  image_encoding_property_->addOption("bgr8", 2);
  image_encoding_property_->addOption("bgra8", 3);
  image_encoding_property_->addOption("mono8", 4);
  image_encoding_property_->addOption("mono16", 5);

  near_clip_property_ = new FloatProperty("Near Clip Distance", 0.01, "Set the near clip distance",
      this, SLOT(updateNearClipDistance()));
  near_clip_property_->setMin(0.01);
}

CameraPub::~CameraPub()
{
  if (initialized())
  {
    render_texture_->removeListener(this);

    unsubscribe();

    context_->visibilityBits()->freeBits(vis_bit_);
  }
}

bool CameraPub::triggerCallback(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res)
{
    res.success = video_publisher_->is_active();
    if (res.success)
    {
      trigger_activated_ = true;
      res.message = "New image will be published on: " + video_publisher_->get_topic();
    }
    else
    {
      res.message = "Image publisher not configured";
    }
  return true;
}

void CameraPub::onInitialize()
{
  Display::onInitialize();

  count_++;

  topic_property_->initialize(context_->getRosNodeAbstraction());
  camera_info_property_->initialize(context_->getRosNodeAbstraction());

  video_publisher_ = new video_export::VideoPublisher(context_->getRosNodeAbstraction().lock()->get_raw_node());

  topic_qos_property_->initialize(
      [this](rclcpp::QoS profile)
      {
        topic_qos_profile_ = profile;
        updateTopic();
      });
  camera_info_qos_property_->initialize(
      [this](rclcpp::QoS profile)
      {
        camera_info_qos_profile_ = profile;
        updateTopic();
      });

  std::stringstream ss;
  ss << "RvizCameraPubCamera" << count_;
  camera_ = context_->getSceneManager()->createCamera(ss.str());
  camera_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  camera_node_->attachObject(camera_);

  std::stringstream ss_tex;
  ss_tex << "RttTexInit" << count_;
  RCLCPP_INFO_STREAM(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(), ss_tex.str());

  // render to texture
  rtt_texture_ = Ogre::TextureManager::getSingleton().createManual(
      ss_tex.str(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      640, 480,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);
  render_texture_ = rtt_texture_->getBuffer()->getRenderTarget();
  render_texture_->addViewport(camera_);
  render_texture_->getViewport(0)->setClearEveryFrame(true);
  render_texture_->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
  render_texture_->getViewport(0)->setOverlaysEnabled(false);
  render_texture_->setAutoUpdated(false);
  render_texture_->setActive(false);
  render_texture_->addListener(this);

  camera_->setNearClipDistance(0.01f);
  camera_node_->setPosition(0, 10, 15);
  camera_node_->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TS_WORLD);

  // Thought this was optional but the plugin crashes without it
  vis_bit_ = context_->visibilityBits()->allocBit();
  render_texture_->getViewport(0)->setVisibilityMask(vis_bit_);

  visibility_property_ = new DisplayGroupVisibilityProperty(
    vis_bit_, context_->getRootDisplayGroup(), this, "Visibility", true,
    "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon(loadPixmap("package://rviz_default_plugins/icons/visibility.svg", true));

  this->addChild(visibility_property_, 0);
  updateDisplayNamespace();
}

void CameraPub::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void CameraPub::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  // set view flags on all displays
  visibility_property_->update();
}

void CameraPub::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  // Publish the rendered window video stream
  const rclcpp::Time cur_time = context_->getRosNodeAbstraction().lock()->get_raw_node()->now();
  rclcpp::Duration elapsed_duration = cur_time - last_image_publication_time_;
  const float frame_rate = frame_rate_property_->getFloat();
  bool time_is_up = (frame_rate > 0.0) && (elapsed_duration.seconds() > 1.0 / frame_rate);
  // We want frame rate to be unlimited if we enter zero or negative values for frame rate
  if (frame_rate < 0.0)
  {
    time_is_up = true;
  }
  if (!(trigger_activated_ || time_is_up))
  {
    return;
  }
  trigger_activated_ = false;
  last_image_publication_time_ = cur_time;
  render_texture_->getViewport(0)->setBackgroundColour(background_color_property_->getOgreColor());

  std::string frame_id;
  {
    boost::mutex::scoped_lock lock(caminfo_mutex_);
    if (!current_caminfo_)
      return;
    frame_id = current_caminfo_->header.frame_id;
    video_publisher_->camera_info_ = *current_caminfo_;
  }

  int encoding_option = image_encoding_property_->getOptionInt();

  // render_texture_->update();
  video_publisher_->publishFrame(render_texture_, frame_id, encoding_option);
}

void CameraPub::onEnable()
{
  subscribe();
  render_texture_->setActive(true);
}

void CameraPub::onDisable()
{
  render_texture_->setActive(false);
  unsubscribe();
  clear();
}

void CameraPub::subscribe()
{
  if (!isEnabled())
    return;

  std::string topic_name = topic_property_->getTopicStd();
  if (topic_name.empty())
  {
    setStatus(StatusProperty::Error, "Output Topic", "No topic set");
    return;
  }

  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  std::string error;
  // ROS2: rclcpp name check in place of ros::names::validate
  try
  {
    rclcpp::expand_topic_or_service_name(topic_name, node->get_name(), node->get_namespace());
  }
  catch (const std::exception& e)
  {
    error = e.what();
  }
  if (!error.empty())
  {
    setStatus(StatusProperty::Error, "Output Topic", QString(error.c_str()));
    return;
  }


  std::string caminfo_topic = camera_info_property_->getTopicStd();
  if (caminfo_topic.empty())
  {
    setStatus(StatusProperty::Error, "Camera Info", "No topic set");
    return;
  }

  // std::string target_frame = fixed_frame_.toStdString();
  // Display::enableTFFilter(target_frame);


  try
  {
    // caminfo_sub_.subscribe(update_nh_, caminfo_topic, 1);
    caminfo_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        caminfo_topic, camera_info_qos_profile_, std::bind(&CameraPub::caminfoCallback, this, std::placeholders::_1));
    caminfo_sub_topic_ = caminfo_sub_->get_topic_name();
    setStatus(StatusProperty::Ok, "Camera Info", "OK");
  }
  catch (std::exception& e)
  {
    setStatus(StatusProperty::Error, "Camera Info", QString("Error subscribing: ") + e.what());
  }

  video_publisher_->advertise(topic_name, topic_qos_profile_);
  setStatus(StatusProperty::Ok, "Output Topic", "Topic set");
}

void CameraPub::unsubscribe()
{
  video_publisher_->shutdown();
  caminfo_sub_.reset();
}

void CameraPub::forceRender()
{
  force_render_ = true;
  context_->queueRender();
}

void CameraPub::updateQueueSize()
{
}

void CameraPub::updateFrameRate()
{
}

void CameraPub::updateNearClipDistance()
{
}


void CameraPub::updateBackgroundColor()
{
}

void CameraPub::updateDisplayNamespace()
{
  std::string name = namespace_property_->getStdString();
  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // ROS2: same name check as the ROS1 ros::NodeHandle(name)
  try
  {
    if (!name.empty())
    {
      rclcpp::expand_topic_or_service_name(name, node->get_name(), node->get_namespace());
    }
    nh_ = name;
  }
  catch (const std::exception& e)
  {
    setStatus(StatusProperty::Warn, "Display namespace", "Invalid namespace: " + QString(e.what()));
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    return;
  }

  video_publisher_->setNodehandle(nh_);

  // ROS_INFO("New namespace: '%s'", nh_.getNamespace().c_str());
  trigger_service_.reset();
  // ROS1 advertiseService returned an empty service when this node already advertised the name
  const std::string service_name = rclcpp::expand_topic_or_service_name(
      nh_.empty() ? camera_trigger_name_ : nh_ + "/" + camera_trigger_name_,
      node->get_name(), node->get_namespace(), true);
  if (node->get_service_names_and_types_by_node(node->get_name(), node->get_namespace()).count(service_name) == 0)
  {
    trigger_service_ = node->create_service<std_srvs::srv::Trigger>(service_name,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
          triggerCallback(*req, *res);
        });
  }

  /// Check for service name collision
  if (!trigger_service_)
  {
    setStatus(StatusProperty::Warn, "Display namespace",
              "Could not create trigger. Make sure that display namespace is unique!");
    return;
  }

  setStatus(StatusProperty::Ok, "Display namespace", "OK");
  updateTopic();
}

void CameraPub::updateImageEncoding()
{
}

void CameraPub::clear()
{
  force_render_ = true;
  context_->queueRender();

  new_caminfo_ = false;
  current_caminfo_.reset();

  setStatus(StatusProperty::Warn, "Camera Info",
            "No CameraInfo received on [" +
            QString::fromStdString(caminfo_sub_topic_) +
            "].  Topic may not exist.");
  // setStatus(StatusProperty::Warn, "Camera Info", "No CameraInfo received");

  camera_node_->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void CameraPub::update(float wall_dt, float ros_dt)
{
#if 0
  try
  {
#endif
  {
    caminfo_ok_ = updateCamera();
    force_render_ = false;
  }

#if 0
  }
  catch (UnsupportedImageEncoding& e)
  {
    setStatus(StatusProperty::Error, "Image", e.what());
  }
#endif

  if ((caminfo_sub_ ? caminfo_sub_->get_publisher_count() : 0) == 0)
  {
    setStatus(StatusProperty::Warn, "Camera Info",
              "No publishers on [" +
               QString::fromStdString(caminfo_sub_topic_) +
               "].  Topic may not exist.");
  }
  render_texture_->update();
}

bool CameraPub::updateCamera()
{
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  {
    boost::mutex::scoped_lock lock(caminfo_mutex_);
    info = current_caminfo_;
  }

  if (!info)
  {
    return false;
  }

  if (!validateFloats(*info))
  {
    setStatus(StatusProperty::Error, "Camera Info", "Contains invalid floating point values (nans or infs)");
    return false;
  }

  // if we're in 'exact' time mode, only show image if the time is exactly right
  rclcpp::Time rviz_time = context_->getFrameManager()->getTime();
  if (context_->getFrameManager()->getSyncMode() == FrameManagerIface::SyncExact &&
      rviz_time != rclcpp::Time(info->header.stamp, rviz_time.get_clock_type()))
  {
    std::ostringstream s;
    s << "Time-syncing active and no info at timestamp " << rviz_time.seconds() << ".";
    setStatus(StatusProperty::Warn, "Time", s.str().c_str());
    return false;
  }

  // TODO(lucasw) this will make the img vs. texture size code below unnecessary
  if ((info->width != render_texture_->getWidth()) ||
      (info->height != render_texture_->getHeight()))
  {
    std::stringstream ss_tex;
    ss_tex << "RttTex" << count_;
    RCLCPP_INFO_STREAM(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(), ss_tex.str());

    rtt_texture_ = Ogre::TextureManager::getSingleton().createManual(
        ss_tex.str(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        info->width, info->height,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
    render_texture_ = rtt_texture_->getBuffer()->getRenderTarget();
    render_texture_->addViewport(camera_);
    render_texture_->getViewport(0)->setClearEveryFrame(true);
    render_texture_->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
    render_texture_->getViewport(0)->setVisibilityMask(vis_bit_);

    render_texture_->getViewport(0)->setOverlaysEnabled(false);
    render_texture_->setAutoUpdated(false);
    render_texture_->setActive(false);
    render_texture_->addListener(this);
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  const bool success = context_->getFrameManager()->getTransform(
      info->header.frame_id, rclcpp::Time(info->header.stamp, RCL_ROS_TIME), position, orientation);
  if (!success)
  {
    std::string error;
    const bool has_problems = context_->getFrameManager()->transformHasProblems(
        info->header.frame_id,
        rclcpp::Time(info->header.stamp, RCL_ROS_TIME), error);
    if (has_problems)
    {
      setStatus(StatusProperty::Error, "getTransform", error.c_str());
      return false;
    }
  }

  // printf( "CameraPub:updateCamera(): pos = %.2f, %.2f, %.2f.\n", position.x, position.y, position.z );

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  float img_width = info->width;
  float img_height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if (img_width == 0)
  {
    RCLCPP_DEBUG(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                 "Malformed CameraInfo on camera [%s], width = 0", qPrintable(getName()));
    img_width = 640;
  }

  if (img_height == 0)
  {
    RCLCPP_DEBUG(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                 "Malformed CameraInfo on camera [%s], height = 0", qPrintable(getName()));
    img_height = 480;
  }

  if (img_height == 0.0 || img_width == 0.0)
  {
    setStatus(StatusProperty::Error, "Camera Info",
              "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  double fx = info->p[0];
  double fy = info->p[5];

  float win_width = render_texture_->getWidth();
  float win_height = render_texture_->getHeight();
  float zoom_x = 1.0;
  float zoom_y = zoom_x;

  // Preserve aspect ratio
  if (win_width != 0 && win_height != 0)
  {
    float img_aspect = (img_width / fx) / (img_height / fy);
    float win_aspect = win_width / win_height;

    if (img_aspect > win_aspect)
    {
      zoom_y = zoom_y / img_aspect * win_aspect;
    }
    else
    {
      zoom_x = zoom_x / win_aspect * img_aspect;
    }
  }

  // Add the camera's translation relative to the left camera (from P[3]);
  double tx = -1 * (info->p[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (info->p[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);

  if (!validateFloats(position))
  {
    setStatus(StatusProperty::Error, "Camera Info",
        "CameraInfo/P resulted in an invalid position calculation (nans or infs)");
    return false;
  }

  const float near_clip_distance = near_clip_property_->getFloat();

  camera_node_->setPosition(position);
  camera_node_->setOrientation(orientation);
  camera_->setNearClipDistance(near_clip_distance);

  // calculate the projection matrix
  double cx = info->p[2];
  double cy = info->p[6];

  double far_plane = 100;
  double near_plane = near_clip_distance;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

  proj_matrix[0][0] = 2.0 * fx / img_width * zoom_x;
  proj_matrix[1][1] = 2.0 * fy / img_height * zoom_y;

  proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width) * zoom_x;
  proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5) * zoom_y;

  proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

  proj_matrix[3][2] = -1;

  camera_->setCustomProjectionMatrix(true, proj_matrix);

  setStatus(StatusProperty::Ok, "Camera Info", "OK");

#if 0
  static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif

  setStatus(StatusProperty::Ok, "Time", "ok");

  return true;
}

void CameraPub::caminfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  boost::mutex::scoped_lock lock(caminfo_mutex_);
  current_caminfo_ = msg;
  new_caminfo_ = true;
}

void CameraPub::fixedFrameChanged()
{
  std::string targetFrame = fixed_frame_.toStdString();
  Display::fixedFrameChanged();
}

void CameraPub::reset()
{
  Display::reset();
  clear();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::CameraPub, rviz_common::Display)
