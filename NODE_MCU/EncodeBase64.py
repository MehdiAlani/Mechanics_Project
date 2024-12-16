import base64
userpass = "applicationId:applicationSecret"
authString = base64.b64encode(userpass.encode()).decode()
print(authString)