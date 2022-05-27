import rospy
from nav_msgs.msg import Odometry
from datetime import datetime
import json
import boto3
import AWSCredentials

# packet ID ( primary key ID for dynamoDB )
count = 0

# Creating Session With Boto3.
session = boto3.Session(
    aws_access_key_id=AWSCredentials.my_access_key_id,
    aws_secret_access_key=AWSCredentials.my_secret_access_key
)

# Creating S3 Resource From the Session.
s3 = session.resource('s3')


def callback(msg):
    """ get data from ROS

    :param msg: Message packet coming from ROS topic
    :return: No return value
    """

    global count
    message = {
        "ID": str(count),
        "PoseX": str("{0:.5f}".format(msg.pose.pose.position.x)),
        "PoseY": str("{0:.5f}".format(msg.pose.pose.position.y)),
        "PoseZ": str("{0:.5f}".format(msg.pose.pose.position.z)),
        "OrientX": str("{0:.5f}".format(msg.pose.pose.orientation.x)),
        "OrientY": str("{0:.5f}".format(msg.pose.pose.orientation.y)),
        "OrientZ": str("{0:.5f}".format(msg.pose.pose.orientation.z)),
        "OrientW": str("{0:.5f}".format(msg.pose.pose.orientation.w))
    }

    print(f"Producing message {datetime.now()} Message :\n {str(message)}")
    count += 1

    try:
        # Getting AWS S3 response
        response = uploadData2AWS(content=message, object_name="odom_" + str(count) + ".json")

        if response.get('HTTPStatusCode') == 200:
            print('File Uploaded Successfully')
        else:
            print('File Not Uploaded')

    except Exception as e:
        print(e)


def uploadData2AWS(content: dict = None, object_name: str = "test") -> dict:
    """ upload data to AWS S3 bucket

    :param data: ROS message content to be uploaded
    :return: aws object upload response (200 means successfully uploaded, otherwise there are an error)
    """

    # Creating S3 object
    object = s3.Object(AWSCredentials.my_bucket_name, object_name)

    # Uploading content to AWS S3
    result = object.put(Body=json.dumps(content))

    # Getting result
    return result.get('ResponseMetadata')


if __name__ == "__main__":
    rospy.init_node('odomSubscriber', anonymous=True)
    rospy.Subscriber('odom', Odometry, callback)
    rospy.spin()
