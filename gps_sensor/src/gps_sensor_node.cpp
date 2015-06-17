//ds STL
#include <cstdio>
#include <memory>

//ds ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>

int main( int argc, char** argv )
{
    //assert( false );

    //ds pwd info
    std::fflush( stdout);
    std::printf( "(main) launched: %s\n", argv[0] );

    //ds configuration parameters
    const std::string strTopicSubscriberTFBase( "" );
    const std::string strTopicSubscriberTFCurrent( "" );
    const std::string strTopicPublisherPose2D ( "pose2d" );
    const uint32_t uMaximumQueueSize( 10 );
    const double dRateSubscriberHz( 10.0 );

    //ds setup node
    ros::init( argc, argv, "gps_sensor_node" );
    ros::NodeHandle hNode( ros::NodeHandle( "~" ) );

    //ds escape here on failure
    if( !hNode.ok( ) )
    {
        std::printf( "\n(main) ERROR: unable to instantiate node\n" );
        std::printf( "(main) terminated: %s\n", argv[0] );
        std::fflush( stdout );
        return 1;
    }

    //ds log configuration
    std::printf( "(main) ROS node namespace              := '%s'\n", hNode.getNamespace( ).c_str( ) );
    std::printf( "(main) ROS strTopicSubscriberTFBase    := '%s'\n", strTopicSubscriberTFBase.c_str( ) );
    std::printf( "(main) ROS strTopicSubscriberTFCurrent := '%s'\n", strTopicSubscriberTFCurrent.c_str( ) );
    std::printf( "(main) ROS strTopicPublisherPose2D     := '%s'\n", ( hNode.getNamespace( ) + '/' + strTopicPublisherPose2D ).c_str( ) );
    std::printf( "(main) ROS uMaximumQueueSize           := '%u'\n", uMaximumQueueSize );
    std::printf( "(main) ROS dRateSubscriberHz           := '%.1f'\n", dRateSubscriberHz );
    std::fflush( stdout );

    //ds transform subscriber
    tf::TransformListener cListenerTransformation;

    //ds 2d pose republisher
    const ros::Publisher cPublisherPose2D( hNode.advertise< geometry_msgs::Pose2D >( strTopicPublisherPose2D, uMaximumQueueSize ) );

    //ds start listening at set rate
    ros::Rate uRateSubscriber( dRateSubscriberHz );
    while( hNode.ok( ) )
    {
        //ds transform to current position
        tf::StampedTransform cTransformation;

        try
        {
            //ds try to retrieve current transformation
            cListenerTransformation.lookupTransform( strTopicSubscriberTFCurrent, strTopicSubscriberTFBase, ros::Time( 0 ), cTransformation );

            //ds buffer origin and rotation
            const tf::Vector3 vecOrigin( cTransformation.getOrigin( ) );

            //ds pose2d to set
            geometry_msgs::Pose2D msgPose2D;

            //ds set pose information
            msgPose2D.x     = vecOrigin[0];
            msgPose2D.y     = vecOrigin[1];
            msgPose2D.theta = tf::getYaw( cTransformation.getRotation( ) );

            //ds republish transform as 2d pose
            cPublisherPose2D.publish( msgPose2D );
        }
        catch( const tf::TransformException& p_cException )
        {
            //ds not fatal
            std::printf( "(main) unable to lookup transform: %s\n", p_cException.what( ) );
        }

        //ds keep the rate
        uRateSubscriber.sleep( );
    }

    //ds exit
    std::printf( "\n(main) terminated: %s\n", argv[0] );
    std::fflush( stdout);
    return 0;
}
