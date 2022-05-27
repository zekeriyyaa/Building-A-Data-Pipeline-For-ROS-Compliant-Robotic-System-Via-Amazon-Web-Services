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




