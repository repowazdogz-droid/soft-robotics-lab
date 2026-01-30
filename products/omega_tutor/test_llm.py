from openai import OpenAI

print('1. Creating client...')
client = OpenAI(base_url='http://localhost:1234/v1', api_key='not-needed')

print('2. Building messages...')
messages = [
    {'role': 'system', 'content': 'You are a helpful tutor.'},
    {'role': 'user', 'content': 'What is 2+2?'}
]

print('3. Calling API...')
response = client.chat.completions.create(
    model='phi-3-mini-4k-instruct',
    messages=messages,
    max_tokens=500,
    temperature=0.7
)

print('4. Response:', response.choices[0].message.content[:100])
