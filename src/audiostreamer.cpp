#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "audio_common_msgs/AudioData.h"
#include "audio_common_msgs/AudioInfo.h"

namespace audio_transport
{
  class RosGstCapture
  {
    public:
      RosGstCapture()
      {
        _bitrate = 192;

        std::string dst_type;

        // Need to encoding or publish raw wave data
        ros::param::param<std::string>("~format", _format, "mp3");
        ros::param::param<std::string>("~sample_format", _sample_format, "F32LE");

        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192);

        // only available for raw data
        ros::param::param<int>("~channels", _channels, 1);
        ros::param::param<int>("~depth", _depth, 16);
        ros::param::param<int>("~sample_rate", _sample_rate, 16000);

        // The destination of the audio
        ros::param::param<std::string>("~dst", dst_type, "appsink");

        // The source of the audio
        //ros::param::param<std::string>("~src", source_type, "alsasrc");
        std::string device;
        ros::param::param<std::string>("~device", device, "");

        _pub = _nh.advertise<audio_common_msgs::AudioData>("audio", 10, true);
        _pub_info = _nh.advertise<audio_common_msgs::AudioInfo>("audio_info", 1, true);

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error",
                         G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // We create the sink first, just for convenience
        if (dst_type == "appsink")
        {
          _sink = gst_element_factory_make("appsink", "sink");
          g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
          g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
          g_signal_connect( G_OBJECT(_sink), "new-sample",
                            G_CALLBACK(onNewBuffer), this);
        }
        else
        {
          ROS_INFO("file sink to %s", dst_type.c_str());
          _sink = gst_element_factory_make("filesink", "sink");
          g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        _source = gst_element_factory_make("alsasrc", "source");
        // if device isn't specified, it will use the default which is
        // the alsa default source.
        // A valid device will be of the foram hw:0,0 with other numbers
        // than 0 and 0 as are available.
        if (device != "")
        {
          // ghcar *gst_device = device.c_str();
          g_object_set(G_OBJECT(_source), "device", device.c_str(), NULL);
        }

        _filterconvertin = gst_element_factory_make("audioconvert", "filterconvertin");
        _highpass = gst_element_factory_make("audiowsinclimit", "highpass");
        g_object_set(G_OBJECT(_highpass), "cutoff", 20.0f, NULL);
        g_object_set(G_OBJECT(_highpass), "length", 501, NULL); // Default is 101, 501 is a good balance for quality and performance
        g_object_set(G_OBJECT(_highpass), "mode", 1, NULL);  //Set the filter mode to high-pass, 1 = highpass, 0 = lowpass


        _amp1 = gst_element_factory_make("audioamplify", "amp1");
        g_object_set(G_OBJECT(_amp1), "amplification", 42.0f, NULL);
        g_object_set(G_OBJECT(_amp1), "clipping-method", 3, NULL); //No clipping, so that the limiter can be applied afterwards

        _limiter = gst_element_factory_make("rglimiter", "limit");

        _compressor1 = gst_element_factory_make("audiodynamic", "compressor1");
        g_object_set(G_OBJECT(_compressor1), "mode", 0, NULL);  // 0 = compressor, 1 = expander
        g_object_set(G_OBJECT(_compressor1), "characteristics", 1, NULL);  //Set to "soft-knee" mode
        g_object_set(G_OBJECT(_compressor1), "threshold", 0.5f, NULL); 
        g_object_set(G_OBJECT(_compressor1), "ratio", 2.0f, NULL); 

        _compressor2 = gst_element_factory_make("audiodynamic", "compressor2");
        g_object_set(G_OBJECT(_compressor2), "mode", 0, NULL);  // 0 = compressor, 1 = expander
        g_object_set(G_OBJECT(_compressor2), "characteristics", 1, NULL);  //Set to "soft-knee" mode
        g_object_set(G_OBJECT(_compressor2), "threshold", 0.5f, NULL); 
        g_object_set(G_OBJECT(_compressor2), "ratio", 2.0f, NULL); 

        _amp2 = gst_element_factory_make("audioamplify", "amp2");
        g_object_set(G_OBJECT(_amp2), "amplification", 1.5f, NULL);
        g_object_set(G_OBJECT(_amp2), "clipping-method", 0, NULL); // Actual clipping, and a little bit of make up gain after the limiter

        _filterconvertout = gst_element_factory_make("audioconvert", "filterconvertout");

        GstCaps *input_caps;
        input_caps = gst_caps_new_simple("audio/x-raw",
                                   "format", G_TYPE_STRING, "S32LE",
                                   "channels", G_TYPE_INT, 1,
                                   "rate",     G_TYPE_INT, _sample_rate,
                                   "signed",   G_TYPE_BOOLEAN, TRUE,
                                   NULL);

        GstCaps *output_caps;
        output_caps = gst_caps_new_simple("audio/x-raw",
                                   "format", G_TYPE_STRING, _sample_format.c_str(),
                                   "channels", G_TYPE_INT, 1,
                                   "rate",     G_TYPE_INT, _sample_rate,
                                   NULL);


        gboolean link_ok;
        if (_format == "wave") {
          if (dst_type == "appsink") {
            g_object_set( G_OBJECT(_sink), "caps", output_caps, NULL);
            gst_caps_unref(output_caps);

            gst_bin_add_many( GST_BIN(_pipeline), _source, _filterconvertin, _amp1, _amp2, _highpass, _compressor1, _compressor2, _limiter, _filterconvertout, _sink, NULL);
            link_ok = gst_element_link_filtered(_source, _filterconvertin, input_caps);
            link_ok = gst_element_link_many(_filterconvertin, _highpass, _amp1, _limiter, _compressor1, _compressor2, _amp2, _filterconvertout, _sink, NULL);
          } else {
            ROS_ERROR_STREAM("dst_type must be \"appsink\"");
            exitOnMainThread(1);
          }
        } else {
          ROS_ERROR_STREAM("format must be \"wave\"");
          exitOnMainThread(1);
        }

        if (!link_ok) {
          ROS_ERROR_STREAM("Unsupported media type.");
          exitOnMainThread(1);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );

        audio_common_msgs::AudioInfo info_msg;
        info_msg.channels = _channels;
        info_msg.sample_rate = _sample_rate;
        info_msg.sample_format = _sample_format;
        info_msg.bitrate = _bitrate;
        info_msg.coding_format = _format;
        _pub_info.publish(info_msg);
      }

      ~RosGstCapture()
      {
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
      }

      void exitOnMainThread(int code)
      {
        exit(code);
      }

      void publish( const audio_common_msgs::AudioData &msg )
      {
        _pub.publish(msg);
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        audio_common_msgs::AudioData msg;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize( map.size );

        memcpy( &msg.data[0], map.data, map.size );

        // Build a histogram of the audio data, if there are mostly zeros, or mostly all the same value,
        // then, the microphone is probably not plugged in.
        int histogram[256] = {0};
        for (int i = 0; i < map.size; i++) {
          histogram[map.data[i]]++;
        }

        int max_count = 0;
        for (int i = 0; i < 256; i++) {
          if (histogram[i] > max_count) {
            max_count = histogram[i];
          }
        }

        if (max_count > map.size / 2) {
          ROS_ERROR_STREAM("Received mostly zeros or all same data.  This is probably a bad connection to the microphone.");
          return GST_FLOW_ERROR;
        }

        //ROS_INFO("Buf value 0 %f", ((float *)map.data)[0]);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        server->publish(msg);

        return GST_FLOW_OK;
      }

      static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        ROS_ERROR_STREAM("gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
      }

    private:
      ros::NodeHandle _nh;
      ros::Publisher _pub;
      ros::Publisher _pub_info;

      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_filterconvertin, *_highpass, *_compressor1, *_compressor2, *_amp1, *_amp2, *_limiter, *_clipper, *_filter, *_filterconvertout, *_sink;
      GstBus *_bus;
      int _bitrate, _channels, _depth, _sample_rate;
      GMainLoop *_loop;
      std::string _format, _sample_format;
  };
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_capture");
  gst_init(&argc, &argv);

  audio_transport::RosGstCapture server;
  ros::spin();
}
