import ast
from pathlib import Path

root=Path(__file__).resolve().parents[1]

class RemoveSiblingImports(ast.NodeTransformer):
    def visit_ImportFrom(self,node):
        return None if node.module in ('mcea_read','mcea_direction_test','simulate_ag') else node

parts=[]
for filename,names in [
    ('mcea_read.py',{'crc16','Reader'}),
    ('mcea_direction_test.py',{'Device','validate','bounds'}),
    ('simulate_ag.py',{'motion_profile','displacement','build_plan'}),
    ('ag_trial.py',None),
]:
    tree=ast.parse((root/filename).read_text(encoding='utf-8'))
    tree.body=[n for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom,ast.Assign,ast.AnnAssign))
               or isinstance(n,(ast.FunctionDef,ast.ClassDef)) and (names is None or n.name in names)]
    tree=RemoveSiblingImports().visit(tree)
    ast.fix_missing_locations(tree)
    parts.append(ast.unparse(tree))

header='''"""A-G manually started, continuous single-cycle hardware runner.
Robot 192.168.21.5; cylinder COM4,115200,8N1,id1.
Hold the orientation captured from hardware on 2026-11-12 13:10:55 UTC+8.
No rotation-to-zero stage. No joint lock. Robot30/cylinder1 mm/s.
After interruption, RECOVER confirms continuous return-to-start preparation and one new cycle.
Dependencies: pyserial==3.5, ur_rtde==1.6.5.
Importing this file never connects or moves hardware.
User launch: python ag_run.py (one RUN confirmation, then continuous execution).
Read only: --check. Offline preview (starting at A): --preview.
Generated from reviewed local trial modules; hardware motion not validated.
"""
'''
tail='''
if __name__ == '__main__':
    if len(sys.argv) == 1:
        sys.argv.extend(['--run'])
    raise SystemExit(main())
'''
output=root/'ag_run.py'
output.write_text(header+'\n\n'.join(parts)+tail,encoding='utf-8')
ast.parse(output.read_text(encoding='utf-8'))
print('Updated',output)
