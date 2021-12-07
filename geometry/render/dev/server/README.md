# Drake REST render API documentation

This document briefly describes the REST render API.
All methods use POST and expect `multipart/form-data`.

## `/upload`

Field `data`:
  The glTF contents.
  Should be sent as if from `<input type="file" name="data">`.
Field `mode`:
  Optional.
  Specifies whether the file is binary/`.glb` (`mode`=`2`),
  text/`.glft` (`mode`=`2`), or whether the server should guess
  (`mode`=`2` or not specified).
  If present, the file name will be used to guess;
  otherwise, text/`.glft` is assumed.

Returns a JSON containing the SHA256 hash of the uploaded file on success.

### cURL example

    curl -X POST -F 'data=@path/to/example.glb' http://127.0.0.1:5000/upload

## `/render`

Field `id`:
  Hash of a previously uploaded glTF to be rendered.
Field `width`:
  Optional.
  Width of the desired rendered image. Defaults to 1920.
Field `height`:
  Optional.
  Height of the desired rendered image. Defaults to width / 1.5.

Returns a PNG image on success.

### cURL example

    curl -X POST -F 'id=<sha>' http://127.0.0.1:5000/render
