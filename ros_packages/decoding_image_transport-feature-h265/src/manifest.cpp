#include <decoding_image_transport/decoding_subscribers.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(decoding_image_transport::H264Subscriber, image_transport::SubscriberPlugin);
PLUGINLIB_EXPORT_CLASS(decoding_image_transport::H265Subscriber, image_transport::SubscriberPlugin);