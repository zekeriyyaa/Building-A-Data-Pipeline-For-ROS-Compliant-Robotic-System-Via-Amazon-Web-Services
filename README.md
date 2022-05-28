# Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services

Robotic systems are equipped with many sensors and produce large-scale data to reflect the system state, recognize the environment or etc. It is possible to extract meaningful analysis to be fed into to system back by analyzing these data. Considering the importance of data, the process of collecting data without any loss from sensors is of great importance. 
<br /><br />During this process, there may be some challenges such as:
- Resource management and scalability
- Data durability and portability
- Security concerns

Cloud-based solutions can be an alternative way to overcome these problems. In this project, a data pipeline structure for robotic systems is presented as shown below. Thus, it is aimed to collect data without any loss, store it in a cloud-based database and even analyze it.

#### The story of application: 
1. Data is created and sent to Amazon S3 to be stored.
2. Amazon S3 event triggers the Amazon Lambda Function.
3. Amazon Lambda Function takes object content from S3 and sends it to Amazon Dynamodb.
<p align="center" width="100%" >
    <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Architecture.PNG"> 
</p>

Follow the given steps:<br/>
1. Prepare the robotic environment
2. Define Identity and Access Management (IAM) Policy & Role
3. Prepare AWS Lambda Function & Code
4. Prepare AWS S3 Bucket & Event Trigger
5. Prepare AWS DynamoDb Table
6. Run The Demo 

### 1. Prepare the robotic environment
You can utilize this [URL](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04)

### 2. Define Identity and Access Management (IAM) Policy & Role
First we need to create an appropriate policy that covers both S3 and DynamoDB permissions. Actually we only need permission to read from S3 and write to DynamoDB. Additionally, we can add CloudWatch permission. Follow the given way. 
>:exclamation: You can find all datails of policy creation process in the [/img](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/tree/main/img) folder.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/IAMCompletePolicy.PNG"">
</p>

After you create the policy, you must define a role with the policy permission you just defined. Follow the given steps:
1. Select trusted entity: AWS Service
2. Select Use Case: Lambda
3. Add Permission: Select the policy you just create
4. Name and create your role 

>:exclamation: You can find all datails of policy creation process in the [/img](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/tree/main/img) folder.

### 3. Create AWS Lambda Function & Code
We will use the lambda function to receive data content from S3 and store it to Dynamodb. First, create a lambda function as given below. Be careful to select an existing permission role that you created the previous step.
>:exclamation: You can find all datails of policy creation process in the [/img](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/tree/main/img) folder.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Lambda1.PNG" >
</p>

#### Check Lambda Function Permission
After you create a lambda function, you can check if everything is alright by using the permissions setting under the configuration section as shown below. It is expected that the function has three different levels of permission to access S3, Dynamodb, and CloudWatch.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Lambda2.PNG" >
</p>

#### Write Lambda Function Code
Write a [lambda function](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/lambda_function.py) that is triggered by Amazon S3 when a new object is created. This function gets the JSON content of the object and sends it into an Amazon Dynamodb table.


```python3
import json
import boto3

s3Client = boto3.client("s3")
dynamoDBClient = boto3.resource("dynamodb")


def lambda_handler(event, context):
    # Get our bucket and file_name from event
    bucket = event["Records"][0]["s3"]["bucket"]["name"]
    key = event["Records"][0]["s3"]["object"]["key"]

    print("bucket name: ", bucket, "\nkey name: ", key)

    table_name = "robotic"
    # Get dynamodb table
    table = dynamoDBClient.Table(table_name)

    print("event => ", event)

    try:
        response = s3Client.get_object(Bucket=bucket, Key=key)
        content = response['Body'].read()
        print("content => ", content)
        content = json.loads(content)

        response = table.put_item(Item=content)
        return {
            'statusCode': 200,
            'body': json.dumps('Successfully !!')
        }

    except Exception as e:
        print(e)
        print(
            'Error while getting object {} from bucket {} and uploading to {}. \nMake sure your object, bucket or dynamodb table is correct ! '.format(
                key, bucket, table_name))
        raise e
```
#### Test & Deploy Lambda Function
Once you create a lambda function, AWS allows you to test it with your custom configurations. Follow the given steps to test the Lambda function:
1. Go to the Test Section
2. Create a new test event
3. Name your test event
4. Select S3 Put template and customize it for your system
5. Save and Test
You can use the following example.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Lambda4.PNG" >
</p>

S3 Put template includes the following information. You have to edit __s3->bucket->name__ and __object->key__ for your system. You can accesss all content [here](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/AWSLambdaTest.json).

```json
{
  "Records": [
    {
      "eventVersion": "2.0",
      "eventSource": "aws:s3",
      "awsRegion": "us-east-1",
      "eventTime": "1970-01-01T00:00:00.000Z",
      "eventName": "ObjectCreated:Put",
      "userIdentity": {
        "principalId": "EXAMPLE"
      },
      "requestParameters": {
        "sourceIPAddress": "127.0.0.1"
      },
      "responseElements": {
        "x-amz-request-id": "EXAMPLE123456789",
        "x-amz-id-2": "EXAMPLE123/5678abcdefghijklambdaisawesome/mnopqrstuvwxyzABCDEFGH"
      },
      "s3": {
        "s3SchemaVersion": "1.0",
        "configurationId": "testConfigRule",
        "bucket": {
          "name": "zd-aws-robotic",
          "ownerIdentity": {
            "principalId": "EXAMPLE"
          },
          "arn": "arn:aws:s3:::example-bucket"
        },
        "object": {
          "key": "odom_1.json",
          "size": 1024,
          "eTag": "0123456789abcdef0123456789abcdef",
          "sequencer": "0A1B2C3D4E5F678901"
        }
      }
    }
  ]
}
```
Finally, you can test your Lambda function with the test content and you will get the response as follow:
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Lambda5.PNG" >
</p>

### 4. Create AWS S3 Bucket & Event Trigger
We create a bucket to store the coming messages into Amazon S3 with. Whenever a new object is created, the bucket triggers a lambda function that we created in the previous step. Follow the given steps:
1. Create a bucket with default settings
2. Open the bucket and go to the Properties section
3. Create a new event notification by selecting "all object create events"
4. Specify the destination as "Lambda function" and select the lambda function that you created in the previous step.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/S3Bucket4.PNG" width="45%" height="300px">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/S3Bucket5.PNG" width="45%" height="300px">
</p>

Finally, you will get the same view as follow:
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/S3Bucket2.PNG" >
</p>

### 5. Create AWS DynamoDb Table
We need to create a Dynamodb table that can store the contents of our robotics data. We specify a partition key as the ID of the content as shown below. We don't have to specify all attributes one by one to store data because dynamodb automatically creates attributes that your S3 object included. 
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/DynamoDB1.PNG" >
</p>

### 6. Run The Demo
Follow the given steps to start a demo:
1. Be sure that you specify your credentials in [AWSCredentials.py](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/AWSCredentials.py)
2. Open a terminal and start ROS by using "roscore" command.
3. Run the [odomPublisher.py](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/odomPublisher.py) to publish the odometry data via "odom" topic. 
```python3
#!/usr/bin/env python3
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(0.2)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
```

5. Run the [sendRoboticData2AWSS3.py](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/sendRoboticData2AWSS3.py) to subscribe "odom" topic and send the data to Amazon S3.
```python3
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
```

After everything is ready you can see the content of robotic data send to Amazon S3 on your terminal as follow:

<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/uploadData2AWSS3.png" >
</p>

Consequently, your robotic data is automatically stored in Dynamodb. When you check your table you will get the same view as follow:

<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/DynamoDB2.PNG" >
</p>

