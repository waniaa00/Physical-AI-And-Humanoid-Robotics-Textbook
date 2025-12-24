"""Fix absolute backend imports to relative imports"""
import os
import re

# Directories to fix (excluding .venv)
DIRS_TO_FIX = [
    'services',
    'repositories',
    'middleware',
    'models',
    'utils',
    '.'  # main.py and other root files
]

def fix_imports_in_file(filepath):
    """Replace 'from backend.' with 'from ' in a single file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        # Replace from backend. with from
        new_content = content.replace('from backend.', 'from ')

        if new_content != content:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(new_content)
            print(f"Fixed: {filepath}")
            return True
        return False
    except Exception as e:
        print(f"Error fixing {filepath}: {e}")
        return False

def main():
    fixed_count = 0
    for dir_name in DIRS_TO_FIX:
        dir_path = os.path.join(os.path.dirname(__file__), dir_name)
        if not os.path.exists(dir_path):
            continue

        # Handle single files in current directory
        if dir_name == '.':
            for filename in os.listdir(dir_path):
                if filename.endswith('.py') and filename != 'fix_imports.py':
                    filepath = os.path.join(dir_path, filename)
                    if os.path.isfile(filepath):
                        if fix_imports_in_file(filepath):
                            fixed_count += 1
        else:
            # Handle directories
            for root, dirs, files in os.walk(dir_path):
                for filename in files:
                    if filename.endswith('.py'):
                        filepath = os.path.join(root, filename)
                        if fix_imports_in_file(filepath):
                            fixed_count += 1

    print(f"\nTotal files fixed: {fixed_count}")

if __name__ == '__main__':
    main()
