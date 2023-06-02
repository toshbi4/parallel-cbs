# parallel-cbs
Parallel realization of Conflict Based Search algorithm

Tested on Ubuntu 20.04

Build
*****
<pre><code>mkdir build
cd build
cmake ..
make -j$(nproc)
</code></pre>

Run Example
***********
One thread
<pre><code>./cbs -i ../maps/shuttles10.yaml -o output.yaml --threads 1
</code></pre>

Three threads
<pre><code>./cbs -i ../maps/shuttles10.yaml -o output.yaml --threads 3
</code></pre>

etc.
