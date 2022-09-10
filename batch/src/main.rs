use std::env;
use std::path::Path;

#[tokio::main]
async fn main() {
    let job_idx = env::var("AWS_BATCH_JOB_ARRAY_INDEX")
        .expect("AWS_BATCH_JOB_ARRAY_INDEX env var missing")
        .parse::<usize>()
        .expect("AWS_BATCH_JOB_ARRAY_INDEX parsing failed");

    let cfd_repo = env::var("CFD_REPO").expect("CFD_REPO env var missing");

    let cfd_case = asms::CFD_CASES[job_idx];
    let path = Path::new(&cfd_repo).join(cfd_case);
    asms::model(path.to_str().expect("Failed to convert path to &str")).await.expect(&format!("Failed to run ASMS model for {cfd_case}"));
}
