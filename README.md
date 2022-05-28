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

Follow the given steps:
0. Prepare the robotic environment
1. Define Identity and Access Management (IAM) Policy & Role
2. Prepare AWS Lambda Function & Code
3. Prepare AWS S3 Bucket & Event Trigger
4. Prepare AWS DynamoDb Table
5. Run The Demo 

### 0. Prepare the robotic environment
You can utilize this [URL](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04)

### 1. Define Identity and Access Management (IAM) Policy & Role
First we need to create an appropriate policy that covers both S3 and DynamoDB permissions. Actually we only need permission to read from S3 and write to DynamoDB. Additionally, we can add CloudWatch permission. Follow the given way. 
>:exclamation: You can find all datails of policy creation process in the [/img/IAM**](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/tree/main/img) folder.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/IAMCompletePolicy.PNG" width="60%">
</p>

After you create the policy, you must define a role with the policy permission you just defined. Follow the given steps:
1. Select trusted entity: AWS Service
2. Select Use Case: Lambda
3. Add Permission: Select the policy you just create
4. Name and create your role 

>:exclamation: You can find all datails of policy creation process in the [/img/IAM**](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/tree/main/img) folder.

### 2. Create AWS Lambda Function & Code
We will use the lambda function to receive data content from S3 and store it to Dynamodb. First, create a lambda function as given below. Be careful to select an existing permission role that you created the previous step.
>:exclamation: You can find all datails of policy creation process in the [/img/Lambda**](https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/tree/main/img) folder.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Lambda1.PNG" width="60%">
</p>

#### Check Lambda Function Permission
After you create a lambda function, you can check if everything is alright by using the permissions setting under the configuration section as shown below. It is expected that the function has three different levels of permission to access S3, Dynamodb, and CloudWatch.
<p align="center" width="100%">
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Lambda2.PNG" width="60%">
</p>

#### Write Lambda Function
Write a function that is triggered by Amazon S3 when a new object is created. This function gets the JSON content of the object and sends it into an Amazon Dynamodb table.


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
  <img src="https://github.com/zekeriyyaa/Building-Robotic-Data-Pipeline-Via-Amazon-Web-Services/blob/main/img/Lambda4.PNG" width="60%">
</p>

S3 Put template includes the following information. You have to edit __bucket->name__ and __object->key__ for your system.

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

### 3. Create AWS S3 Bucket & Event Trigger
### 4. Create AWS DynamoDb Table
### 5. Run The Demo 

