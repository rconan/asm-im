
#[tokio::main]
async fn main() -> anyhow::Result<()> {

    asms::model("/fsx/CASES/zen30az000_OS7").await?;

    Ok(())
}
