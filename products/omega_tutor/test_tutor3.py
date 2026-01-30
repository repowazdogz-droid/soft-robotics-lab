import sys
sys.path.insert(0, '..')

print('1. Importing...', flush=True)
from core.tutor_engine import TutorEngine

print('2. Creating engine...', flush=True)
engine = TutorEngine()
print('3. Engine object created', flush=True)

print('4. Checking attributes...', flush=True)
print('   Has _local_model_id:', hasattr(engine, '_local_model_id'), flush=True)

print('5. Getting model id...', flush=True)
model_id = engine._local_model_id
print('   Model ID:', model_id, flush=True)

print('6. Skipping teach(), just testing LLM directly...', flush=True)

from openai import OpenAI
client = OpenAI(base_url='http://localhost:1234/v1', api_key='not-needed')
response = client.chat.completions.create(
    model=model_id,
    messages=[{'role': 'user', 'content': 'Say hi'}],
    max_tokens=50
)
print('7. Direct LLM response:', response.choices[0].message.content, flush=True)

print('8. Now testing teach()...', flush=True)
result = engine.teach('What is 2+2?', 'adult')
print('9. Teach result:', result.get('explanation', '')[:100], flush=True)
