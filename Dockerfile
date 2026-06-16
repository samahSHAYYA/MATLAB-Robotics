FROM matlab-runtime:R2025a

WORKDIR /workspace

COPY . .

RUN addpath('.'); savepath;

CMD ["matlab", "-batch", "addpath('tests'); results = runtests('tests'); disp(results);"]
