import sys
sys.path.insert(0, 'research_system')
from research_memory import ResearchMemory
import fitz

print('PyMuPDF version:', fitz.version)
doc = fitz.open('temp_uploads/1470623.pdf')
print(f'PDF pages: {len(doc)}')
print(f'First page: {doc[0].get_text()[:300]}')
doc.close()

memory = ResearchMemory('test_lab')
count = memory.ingest('temp_uploads/1470623.pdf', project='test')
print(f'Ingested: {count} chunks')
print(memory.get_stats())
