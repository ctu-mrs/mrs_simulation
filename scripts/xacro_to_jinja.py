#!/usr/bin/python3

import sys

def load_xacro_content(filename):
    print('Reading', filename)
    with open(filename, 'r', encoding='utf-8') as f:
        return f.readlines()

def write_xacro_content(data, filename):
    print('Writing into file', filename)
    with open(filename, 'w', encoding='utf-8') as f:
        f.writelines(data)

def unify_quotes(lines):
    
    new_lines = []
    for line in lines:
        new_lines.append(line.replace('"', "'"))
    return new_lines

def remove_xacro_properties(lines):
    
    new_lines = []

    block_end = True
    
    for line in lines:
        if 'xacro:property' in line:
            block_end = False
            continue

        if not block_end:
            block_end = '/>' in line or '</xacro:property>' in line
        
        if not block_end:
            continue

        new_lines.append(line)
    return new_lines


def replace_xacro_args(lines):

    new_lines = []

    for line in lines:
        new_line = line.replace("<xacro:arg name='", "{%- set ")
        new_line = new_line.replace(" default='", " = ")
        new_line = new_line.replace("'", "")
        new_line = new_line.replace("/>", "-%}")
        new_lines.append(new_line)

    return new_lines

def replace_comments(lines):

    new_lines = []
    
    for line in lines:

        new_line = line.replace("<!--}-->", "")
        new_line = new_line.replace("<!--{", "<!--")

        new_line = new_line.replace("<!--", "{#")
        new_line = new_line.replace("-->", "#}")
        new_lines.append(new_line)
    return new_lines


if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        exit()
    
    data = load_xacro_content(filename)

    data = unify_quotes(data)
    data = remove_xacro_properties(data)
    data = replace_xacro_args(data)
    data = replace_comments(data)

    write_xacro_content(data, filename + '.sdf.jinja')



