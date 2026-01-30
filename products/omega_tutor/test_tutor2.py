import sys
sys.path.insert(0, '..')

print('1. Importing...', flush=True)
from core.tutor_engine import TutorEngine

print('2. Creating engine...', flush=True)
engine = TutorEngine()

print('3. Engine created', flush=True)
print('   Model:', engine._local_model_id, flush=True)

print('4. About to call teach()...', flush=True)
print('   This may take 10-30 seconds...', flush=True)

result = engine.teach('What is 2+2?', 'adult')

print('5. Got result:', flush=True)
print('   Explanation:', result.get('explanation', 'none')[:100], flush=True)
