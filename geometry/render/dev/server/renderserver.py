#!/usr/bin/env python

import argparse
import atexit
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap

from hashlib import sha256

from flask import jsonify, Flask, request, send_file

app = Flask(__name__)
renderer = None

scratch_dir = None

scenes = {}  # sha: is_binary


def cleanup():
    if scratch_dir is not None:
        shutil.rmtree(scratch_dir)


def tempnam(suffix='', dir=None):
    if dir is None:
        dir = scratch_dir

    return tempfile.mktemp(suffix=suffix, dir=dir)


def compute_hash(path):
    block_size = 1024 * 1024

    h = sha256()
    with open(path, 'rb') as f:
        while True:
            data = f.read(block_size)
            if not len(data):
                break

            h.update(data)

    return h


def scene_path(sha):
    is_binary = scenes.get(sha, False)
    extension = '.glb' if is_binary else '.gltf'
    return os.path.join(scratch_dir, sha + extension)


@app.route('/')
def root():
    return textwrap.dedent('''\
        <!doctype html>
        <html>
        <body>
        <p><a href="/upload">Upload glTF</a></p>
        <p><a href="/render">Render scene</a></p>
        </body>
        </html>
        ''')


@app.route('/upload', methods=['GET', 'POST'])
def upload():
    if request.method == 'POST':
        if 'data' not in request.files:
            return jsonify(
                isError=True,
                message='Expected key `data`.',
                statusCode=400)

        f = request.files['data']

        try:
            mode = request.form.get('mode', 0)
            if int(mode) == 0:
                if os.path.splitext(f.filename)[1].lower() == '.glb':
                    extension = '.glb'
                    is_binary = True
                else:
                    extension = '.gltf'
                    is_binary = False
            elif int(mode) == 1:
                extension = '.gltf'
                is_binary = False
            elif int(mode) == 2:
                extension = '.glb'
                is_binary = True
            else:
                raise ValueError
        except ValueError:
            return jsonify(
                isError=True,
                message=f'File mode `{mode}` is not valid.',
                statusCode=400)

        temp_path = tempnam()
        f.save(temp_path)

        sha = compute_hash(temp_path).hexdigest()
        filename = f'{sha}{extension}'

        os.rename(temp_path, os.path.join(scratch_dir, filename))

        global scenes
        scenes[sha] = is_binary
        print('accepted', sha)

        return jsonify(
            isError=False,
            sha256=sha,
            statusCode=200)

    return textwrap.dedent('''\
        <!doctype html>
        <html>
        <body>
        <h1>Upload glTF</h1>
        <form method="post" enctype="multipart/form-data">
            <input type="file" name="data">
            <select name="mode">
                <option value="0">Auto-detect</option>
                <option value="1">Text (.gltf)</option>
                <option value="2">Binary (.glb)</option>
            </select>
            <input type="submit" value="Upload">
        </form>
        </body>
        </html>
        ''')


@app.route('/render', methods=['GET', 'POST'])
def render():
    if request.method == 'POST':
        if 'id' not in request.form:
            return jsonify(
                isError=True,
                message='Expected key `id`.',
                statusCode=400)

        try:
            w = request.form.get('width', 1920)
            if int(w) < 0:
                raise ValueError
            w = int(w)
        except ValueError:
            return jsonify(
                isError=True,
                message=f'Requested width `{w}` is not valid.',
                statusCode=400)

        try:
            h = request.form.get('height', w / 1.5)
            if int(h) < 0:
                raise ValueError
            h = int(h)
        except ValueError:
            return jsonify(
                isError=True,
                message=f'Requested height `{h}` is not valid.',
                statusCode=400)

        i = request.form['id']
        input_path = scene_path(i)
        if not os.path.exists(input_path):
            return jsonify(
                isError=True,
                message=f'Scene `{i}` not found.',
                statusCode=404)

        output_path = tempnam(suffix='.png')
        result = subprocess.run([
            renderer,
            f'--output={output_path}',
            f'--resolution={w},{h}',
            input_path])
        if result.returncode != 0:
            print(f'render failed: renderer returned {result.returncode}')
            return jsonify(
                isError=True,
                message='Rendering failed.',
                statusCode=503)

        print('rendered', i, f'at {w}x{h} to', output_path)
        return send_file(output_path, mimetype='image/png')

    options = [f'<option value="{x}">{x}</option>' for x in scenes.keys()]
    size_attrs = 'type="number" name="width" min="16" max="65535"'

    return textwrap.dedent('''\
        <!doctype html>
        <html>
        <body>
        <h1>Render scene</h1>
        <form method="post" enctype="multipart/form-data">
            <select name="id">
                ''' + '\n        '.join(options) + '''
            </select>
            <br/>
            <input ''' + size_attrs + ''' value="1920">
            x
            <input ''' + size_attrs + ''' value="1280">
            <br/>
            <input type="submit" value="Render">
        </form>
        </body>
        </html>
        ''')


def main(args):
    global renderer
    global scratch_dir

    atexit.register(cleanup)
    scratch_dir = tempfile.mkdtemp()

    parser = argparse.ArgumentParser(
        description='Drake VTK render test server.')
    parser.add_argument(
        'renderer',
        help='path to render utility (f3d)')

    options = parser.parse_args(args)
    renderer = options.renderer

    # TODO handle args
    app.run()


if __name__ == '__main__':
    main(sys.argv[1:])
