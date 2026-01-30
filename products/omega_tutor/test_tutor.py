import sys
sys.path.insert(0, '..')

print('1. Importing TutorEngine...')
from core.tutor_engine import TutorEngine

print('2. Creating engine...')
engine = TutorEngine()

print('3. Engine created. Model:', engine._local_model_id)
print('4. Calling teach()...')

try:
    result = engine.teach('What is 2+2?', 'adult')
    print('5. Result:', result)
except Exception as e:
    print('5. Error:', e)
    import traceback
    traceback.print_exc()
