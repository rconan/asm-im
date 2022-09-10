FROM rust as build

RUN apt-get update && apt-get -y install clang cmake

ADD Cargo.toml /test/
COPY ./src  /test/src
COPY ./batch  /test/batch
WORKDIR /test 

COPY modal_state_space_model_2ndOrder.zip /
ENV FEM_REPO="/"
RUN cargo build --release -p asms-batch

FROM rust:slim

COPY --from=build /test/target/release/asms-batch asms