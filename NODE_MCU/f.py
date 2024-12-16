import requests
from requests.auth import HTTPBasicAuth

# API URL and credentials
url = "https://api.astronomyapi.com/api/v2/bodies/positions"
app_id = "f659e764-dcc6-4fc5-b795-e897cdeed780"  
app_secret = "6e6ae65766f2893a6aa750a3c58556a919a0c7d3affc5319147bcdbd884089c41c124807f7eda3e1398ab16a42ddc274b1a1d64f1f9cddac8c25a7de1d58a2575e39e8d1c7a1636642d22d8889da5fab041d9ea96c07d5f85637c32f6879cc6e07202a3b65bf6c29598d0dcd316703d0"  # Replace with your App Secret

# Define request parameters
params = {
    "latitude": 36.833901,    # Your latitude
    "longitude": 10.203795,   # Your longitude
    "elevation": 10,          # Your elevation (in meters)
    "from_date": "2024-12-15",  # Date range (start)
    "to_date": "2024-12-16",    # Date range (end)
    "time": "12:41:00",         # Specific time of interest
}

# Make the request with Basic Authentication
with requests.Session() as session:
    request = requests.Request(
        "GET",
        url,
        params=params,
        auth=HTTPBasicAuth(app_id, app_secret),
    )
    prepared_request = session.prepare_request(request)

    # Print the raw request details
    print("REQUEST URL:")
    print(prepared_request.url)
    print("\nREQUEST HEADERS:")
    print(prepared_request.headers)
    print("\nREQUEST BODY:")
    print(prepared_request.body)

    # Send the actual request
    response = session.send(prepared_request)

    # Print the response
    print("\nRESPONSE:")
    if response.status_code == 200:
        print("Response Data:", response.json())
    else:
        print("Error:", response.status_code, response.json())
