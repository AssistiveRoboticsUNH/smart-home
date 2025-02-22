import requests
from bs4 import BeautifulSoup

def get_weather(location='USNH0079:1:US'):
    # Example URL for a weather site (adjust as needed)
    url = f"https://www.weather.com/weather/today/l/{location}"
    
    # Send GET request to the website
    response = requests.get(url)
    if response.status_code == 200:
        # Parse the HTML content
        soup = BeautifulSoup(response.text, 'html.parser')
        
        # Find temperature element (HTML structure may vary)
        temp_element = soup.find('span', class_='CurrentConditions--tempValue--zUBSz')
        desc_element = soup.find('div', class_='CurrentConditions--phraseValue---VS-k')
        
        if temp_element and desc_element:
            temperature = temp_element.text
            description = desc_element.text
            # print(f"Current temperature: {temperature}")
            # print(f"Weather description: {description}")
        else:
            print("Could not find weather data. The page structure might have changed.")
    else:
        print("Failed to retrieve data. Check the URL or location code.")
    response_txt = f'Current temperature is {temperature} degrees. And its {description}'
    print(response_txt)
    return response_txt

# get_weather('USNH0079:1:US')
