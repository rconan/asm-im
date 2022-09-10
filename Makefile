build:
	docker build -t gmto.im/asms .
run:
	docker run --rm gmto.im/asms
push:
	aws ecr get-login-password --region us-west-2 | docker login --username AWS --password-stdin 378722409401.dkr.ecr.us-west-2.amazonaws.com
	docker tag gmto.im/asms:latest 378722409401.dkr.ecr.us-west-2.amazonaws.com/gmto.im/asms:latest
	docker push 378722409401.dkr.ecr.us-west-2.amazonaws.com/gmto.im/asms:latest
stack:
	aws s3 cp asms.yaml s3://gmto.modeling/stacks/
	aws cloudformation create-stack --stack-name asms --template-url https://s3-us-west-2.amazonaws.com/gmto.modeling/stacks/asms.yaml --region us-west-2
