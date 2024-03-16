Environment Setup
=================

We use `Sphinx <https://www.sphinx-doc.org/>`__ for
building and formatting the html files. To add to
the documentation, we use a combination of VSCode extensions
and terminal commands which allow a quick and simple way
of writing documentation and viewing it live.

Prerequisites
-------------

- `Python 3.12 <https://www.python.org/downloads/>`__ (ensure that it is in path)
- `Make <https://gnuwin32.sourceforge.net/packages/make.htm>`__ (only for windows)
- `Python extension <https://marketplace.visualstudio.com/items?itemName=ms-python.python>`__
- `reStructuredText <https://marketplace.visualstudio.com/items?itemName=lextudio.restructuredtext>`__
- `reStructuredText Syntax <https://marketplace.visualstudio.com/items?itemName=trond-snekvik.simple-rst>`__
- `Live Preview <https://marketplace.visualstudio.com/items?itemName=ms-vscode.live-server>`__
- `File Watcher <https://marketplace.visualstudio.com/items?itemName=appulate.filewatcher>`__

.. note:: 
    Don't forget to add the ``make`` binary to path, it is called during building.

Configuration
-------------

Once all of the extensions are installed, open settings and make sure to set the esbonio paths.

.. code-block:: json

    "esbonio.sphinx.numJobs": 0,
    "esbonio.sphinx.buildDir": "docs\\build",
    "esbonio.sphinx.confDir": "${workspaceFolder}\\docs\\source"

To view the ``rst`` file updates live, we use the ``filewatcher`` and ``livePreview`` extensions together.

.. code-block:: json

    "filewatcher.commands": [
      {
        "event": "onFileChange",
        "match": "\\.rst*",
        "cmd":"${workspaceRoot}\\docs\\make.bat html"
      }
    ],
    "livePreview.defaultPreviewPath": "docs/build/html/index.html",
    "livePreview.previewDebounceDelay": 3000

Additionally, to have the preview automatically change as you type, set the autosave delay around ``1000``

.. code-block:: json

    "files.autoSave": "afterDelay",
    "files.autoSaveDelay": 1000

