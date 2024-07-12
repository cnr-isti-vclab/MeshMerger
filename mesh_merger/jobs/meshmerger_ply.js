//const fetch = require('node-fetch');
const http = require("http");
const host = '0.0.0.0'; // 'localhost'pc-ponchio.isti.cnr.it 
const port = 8080;  //80

const url = require("url");
const fs = require('fs'); 
const ply = require("ply");
const { spawn } = require('child_process');
const { parse } = require('querystring');
const path = require('path'); // Add this to the top where you import modules

const { URL } = require('url');

//const executablePath = "D:\\CHARITY\\code\\mesh_merger_exe\\charity_proj.exe";


let jobs =  {};

//Jobs: { id: job }
//Job: { id, [files], status, progress } //assume folder is just id.
// id: string
// status: downloading | processing | failed | done | on-queue
// progress: integer //number of downloaded files.
// file: { url, path (of the saved file), status (onqueue, downloading, done) }

let downloading = false;
let downloadQueue = [];

let processing = false;
let processingQueue = [];

/* Sequence of operations:

  newFragment(id, url)
    	1) if(!job exists)
  		  create folder
  		  create job, and add to jobs.

  	2) append file to job.files
  	   append file to downloadQueue
  	   append id to processingQueue (this needs to be a function that checks if the id is present and in case skip)

	3) check for downloading and processing and in case start 2 functions process() and download()


//this function is called when some file is downloaded.
async process() {
    1) if processing
    1) set processing true
    2) get first job
    3) if not all is downloaded
	put job back on the queue
	if it there are no other jobs exit (and set processing false)
       else
         wait for run merge, set job status as processing
          when done set job status as done or failed
     4) get to next job)
    
}

async download() {
    1) set downloading true
    2) get first file
    3) wait for downloadFile(file) -> this needs to be a function.
    4) when done call process()
    5) get next file or exit (and set downloading false).
}
*/

/**
 * Handles HTTP requests and routes them based on the request pathname.
 *
 * @param {Object} req - The HTTP request object.(https://nodejs.org/api/http.html)
 * @param {Object} res - The HTTP response object.(https://nodejs.org/api/http.html)
 */

const requestListener =  async function (req, res) {
	
      // Parse the request URL
	const reqUrl = url.parse(req.url, true);
	
	const currentDate = new Date();
	console.log(`Current date: ${currentDate}`);
	console.log('ip:', req.socket.localAddress);
	console.log('url:', req.url);
	console.log('current exection folder:',__dirname)
	
	switch(reqUrl.pathname) {
		case '/fragment':
		{
			let body = '';
			req.on('data', function (data)
			{
				body += data;
			});
            // Parse POST data and process the job
			req.on('end', async function ()
			{
				let post = parse(body);
				newFragment(post.id, post.url);
			})
			res.writeHead(200, { 'Content-Type': 'text/plain' });
			res.end(`OK`);
		}
		break;
		

				
	// Case: '/job' - Handles new job requests
		case '/job': //new job
		{
			// Handle POST data asynchronously
            let body = '';

			req.on('data', function (data)
			{
            body += data;
			});
            // Parse POST data and process the job
			req.on('end', async function ()
			{
            let post = parse(body);
			let jobId = post.jobId;
			let filenames = post.filenames;
			newFragment(post.jobId, post.filenames);

			res.writeHead(200, { 'Content-Type': 'text/plain' });
			res.end(`OK`);
			});
		}
		break;
    	/* Case: '/job/status' - Handles requests for job status 
		and returns the status of a particular job, The parameter is a jobId,
		returns a json with the information of the input meshes, status of processing , timings of the core algorithm*/	
		
		case '/job/status': //parameter jobid
		{
            let jobId = reqUrl.query.jobId;
            if (typeof (jobId) == "undefined" || !jobs[jobId]) {
                res.writeHead(404, { "Content-Type": "text/html" });
				res.end("Job not found")
                return;
            }
            res.setHeader("Content-Type", "application/json");
            res.end(JSON.stringify(jobs[jobId]));
		}
		break;
	
	    case '/testurl':
		{
			res.writeHead(200, { 'Content-Type': 'text/html' });

			var stream = fs.createReadStream("index.html");
			stream.pipe(res);
		}
		break;
		case '/job/download':
		{
			
			//res.sendFile("index.html
			let jobId = reqUrl.query.jobId;
			if (typeof (jobId) == "undefined" || !jobs[jobId]) {
				console.log("not found");
                res.writeHead(404, { "Content-Type": "text/html" });
				res.end("Job not found");
                return;
            }
			
			
			let job = jobs[jobId];
			
			if(typeof(job.output)=="undefined")
			{
				res.writeHead(404, { "Content-Type": "text/html" });
				res.end("Job not ready");
				return;
			}
			res.setHeader('Content-Disposition', `inline; filename="${jobId}.ply"`)
			res.writeHead(200, { 'Content-Type': 'application/ply' });
			
			
			var stream = fs.createReadStream("./" + job.output);
			stream.pipe(res);
		}
		break;
		
		default:
			res.writeHead(404);
			res.end("There is nothing here!");
	}
};

//path is in the form /jobs/{id}/file.ply
function sendMergedMesh(res, path) {
	//TODO match {id} with existing ids
	res.sendFile('.'+ path);
}

async function newFragment(id, url) {
    // Check if the job exists, if not, create a folder and add the job to the jobs object
    if (!jobs[id]) 
	{
		if(!fs.existsSync(id))
		{
           fs.mkdirSync(id); // Create a folder for the job   
		}
		jobs[id] = { id, files: [], status: "on-queue", progress: 0 };
    }
	// position of the file in file array is the filename
    //const filenameMatch = file.url.match(/\/([^\/]+)\/?$/);
    let  file = { url, path:`${id}/${id}-${jobs[id].files.length}.ply`, job:id, status: "on-queue", processed:false }
    // Append the file to the job's files array
    jobs[id].files.push(file);
	

    // Append the job ID to the processing queue if it's not already present
    /*if (!processingQueue.includes(id)) {
        processingQueue.push(id);

        // Check if processing is already ongoing, if not, start it
        if (!processing) {
            await process();
        }
    }*/

    // Append the file to the download queue
    downloadQueue.push(file);

    // Check if downloading is already ongoing, if not, start it
    if (!downloading) {
        await download();
    }
}

async function process() {
 console.log("processing:", processing)
 if(processing)
	 return
 // Set processing flag to true
    processing = true;

    // Continue processing until there are jobs in the queue
    while (processingQueue.length > 0) {
        // Get the first job ID from the processing queue
        const job = processingQueue.shift();
        console.log(job)
   
        // Check if all files for the job have been downloaded
        const allFilesDownloaded = job.files.every(file => file.status === "done");
		console.log(allFilesDownloaded)

        if (!allFilesDownloaded) 
		{
            // Put the job back on the queue
            /*processingQueue.push(jobId);

            // Exit the loop if there are no other jobs in the queue
            if (processingQueue.length === 1)
			{
                break;
            } 
			else 
			{*/
                // Move to the next job
                //processingQueue.shift();
                continue;

        }

        // Set job status as processing
        job.status = "processing";

        // Run merge (assuming it's a synchronous function)
        job.status = await runMerge(job);

    }

    // Set processing flag to false
    processing = false;
}

async function download() {
	
	if(downloading)
	   return
    // Set downloading flag to true
    downloading = true;

    // Continue downloading until there are files in the queue
    while (downloadQueue.length > 0) {
        // Get the first file from the download queue
        const file = downloadQueue.shift();

        // Get the filename from the URL
      
        const filename = file.path;

        // Check if the file already exists
        if (fs.existsSync(filename)) 
		{
			
			console.log( "file already exist" + file.url)
     
        } 
	
            try 
			{
                const fetchRes = await fetch(file.url);
                if (!fetchRes.ok) 
				{
                    throw new Error(`Failed to fetch PLY data: ${fetchRes.status} ${fetchRes.statusText}`);
                }

                // Calculate total download size
                const contentLength = fetchRes.headers.get('content-length');
                let downloadedBytes = 0;

                // Use fetchRes.body as a ReadableStream to track progress
                const reader = fetchRes.body.getReader();

                // Write the response to file
                const fileStream = fs.createWriteStream(filename);

                while (true) 
				{
                    const { done, value } = await reader.read();

                    if (done) 
					{
                        break;
                    }

                    // Write chunk to file
                    fileStream.write(value);

                    // Update download progress
                    downloadedBytes += value.length;
                    const progress = (downloadedBytes / contentLength) * 100;
                    console.log(`Download progress: ${progress.toFixed(2)}%`);
                }

                // Close the file stream
                fileStream.end();

                // Mark the file as done
				file.status = "done";
				let job = jobs[file.job]
				job.progress++; // Increment progress for the job
				if(job.files.length >= 2)
				{
					processingQueue.push(job)
					process()
				}
                
            } 
			catch (error) 
			{
				throw 'Error downloading file:'+ error
            }
       
    }

    // Set downloading flag to false
    downloading = false;
}

/**
 * Process a job with specified Job ID and URLs.
 *
 * @param {string} jobId - The unique identifier for the job.
 * @param {Array|string} urls - An array or string containing URLs for processing.
 * @returns {Object} - An object indicating the processing status.
 *   - status: "success" if processing is successful, "error" otherwise.
 *   - msg: An additional message providing details in case of an error.
 */

 async function  jobProcessing(jobId, urls)
 {

			// Extract filenames from POST parameters
			console.log('url for scans:', urls)
			const fileArray = [];
			if(!Array.isArray(urls))
				urls = [urls]
			
			let job = {id:jobId, urls,status:"downloading", progress:0}
			jobs[jobId]= job
			for( let url of urls)
			{
				const filenameMatch = url.match(/\/([^\/]+)\/?$/);
				const filename = filenameMatch ? filenameMatch[1] : null;
				
                fileArray.push(filename);
				console.log('Filename:', filename);

				if (!fs.existsSync(filename))
				{
					 try
					 {
					   const fetchRes = await fetch(url);
					   if (!fetchRes.ok) {
							throw new Error(`Failed to fetch PLY data: ${fetchRes.status} ${fetchRes.statusText}`);
						}
                        // Calculate total download size
					   const contentLength = fetchRes.headers.get('content-length');
					   let downloadedBytes = 0;
					    // Use fetchRes.body as a ReadableStream to track progress
					   const reader = fetchRes.body.getReader();

						// Write the response to file
					   const fileStream = fs.createWriteStream(filename);

					   while (true) 
					   {
							const { done, value } = await reader.read();

							if (done) 
							{
								break;
							}

						   // Write chunk to file
						    fileStream.write(value);

							// Update download progress
						    downloadedBytes += value.length;
							const progress = (downloadedBytes / contentLength) * 100;
							console.log(`Download progress: ${progress.toFixed(2)}%`);
					   }
					   // Close the file stream
					   fileStream.end();
					   
                       console.log('Binary PLY data written to', filename);
						// Use fetchRes.buffer() to get the response body as a Buffer
						//const plyBuffer = Buffer.from(await fetchRes.arrayBuffer());

						// Generate binary PLY and write to file
						
						//await fs.promises.writeFile(filename, plyBuffer);
						//console.log('Binary PLY data written to output_binary.ply');

						//res.end('job-id:', jobId);
					
					} 
					catch (error)
					{
						return {status:"error", msg:"Error processing PLY file: " + url};
						console.error('Error processing PLY file:',error);
						
					}
				}
			}
			job.status = "processing"
			// from here
			const outFileName = `${id}/mergedmesh_${jobId}.ply`;
			fileArray.push(outFileName)
			console.log(fileArray)
			const childProcess = spawn(executablePath, fileArray);

			// Execute the command
			childProcess.stdout.on('data', (data) => {
			  const outputString = data.toString('utf-8');
			  console.log(`stdout: ${outputString}`);
			  job.stdout = outputString
			});

			childProcess.stderr.on('data', (data) => {
			const errorString = data.toString('utf-8');
			console.error(`stderr: ${errorString}`);
			job.stderr = errorString
			  
			});

			childProcess.on('close', (code) => {
			  console.log(`child process exited with code ${code}`);
			  if( code == 0)
				  job.status = "success"
			  else
				  job.status = "fail"
			  
			});
			
		
	return{status:"success"}	
 };

async function runMerge(job)
{   
     return new Promise((resolve,reject)=>
    {
 
		const volumeArg = '-v d:';	
		//let fileArray = job.files.map((f)=> '/jobs/' + f.path)
		
		/*Each file has a processed property.
		123-1.ply  
		123-2.ply  -> 123-2-merged.ply
		123-3.ply  -> 123-3-merged.ply
		123-4.ply  -> 123-4-merged.ply */

        //this should never trigger!
        console.assert(job.files.length) > 2;
		
		let file1, file2, mesh1, mesh2, file3, mesh3;
		let fliporient = true
		
		if(!job.files[0].processed) 
		{
			  file1 = job.files[0];
			  file2 = job.files[1];
			  mesh1 = file1.path;
			  mesh2 = file2.path;
			  console.log("mesh1:",mesh1)
		      console.log("mesh2:",mesh2)
		}
		
		else 
		{
			
		  for(let i = 1; i < job.files.length-1; i++)
		  {
				let current = job.files[i];
				let next = job.files[i+1];
				
				if(current.processed && !next.processed)
				{
					file1 = current;
					mesh1 =   next.path;
					
					file2 = next
					mesh2 = current.path.substr(0, file1.path.length - 4) + "-merged_low_resolution.ply";
					fliporient = false
					break;
				}
	      }
		 console.assert(mesh1);
         console.assert(mesh2);		 
		}

		//console.log("fileArray:", fileArray)
		//const outFileName =  `/jobs/${job.id}/mergedmesh_${job.id}.ply`;
		console.log("file2.path.substr:", file2.path.substr(0, file2.path.length- 4))
		
		let outFileName = file2.path.substr(0, file2.path.length - 4) + "-merged.ply";
		let outRegisFileName = file2.path.substr(0, file2.path.length - 4) + "-merged_low_resolution.ply";
		let outFilePath =  outFileName;
		let outRegisFilePath = outRegisFileName;
	
		console.log("outfilename:",outFileName)
		//fileArray.push(outFileName)
		//console.log(fileArray)
		//let array = [__dirname + ':/jobs', 'mesh_merger', 'sh', '-c', 'chmod 644 /jobs/*.ply && ./charity ' + fileArray.join(' ') ]  
		//let array = [__dirname + ':/jobs', 'mesh_merger', 'sh', '-c', `chmod 644 /jobs/*.ply && ./charity', mesh1, mesh2, outFileName];
		//valid only for external server
		/*let array = [
			  'D:\\CHARITY\\code\\node_js_scripts:/jobs', // Volume mount specification
			  'mesh_merger',              // Command to execute inside the container
			  'sh',                       // Shell to use for the command
			  '-c',                       // Option to indicate the following argument is a command
			  `chmod 644 /jobs/*.ply && ./charity ${fliporient} ${mesh1} ${mesh2} ${outRegisFilePath} ${outFilePath}` // Command to run inside the container with arguments
			];
         
		console.log("array:",array)
	
		let dockerCmd = 'docker';
		let dockerArgs = [
				'run',
				'--rm', // Optionally remove the container after execution
				'-v', // Spread volume arguments
				...array // Your existing fileArray as arguments, if applicable
			];
		console.log(dockerArgs)	
			 // Execute the Docker command

		const dockerProcess = spawn(dockerCmd, dockerArgs);*/
		////////upto here//////
		let array = [
			  '-c',                       // Option to indicate the following argument is a command
			  `../charity ${fliporient} ${mesh1} ${mesh2} ${outRegisFilePath} ${outFilePath}` // Command to run inside the container with arguments
			];
		const dockerProcess = spawn("sh", array)	
		file1.processed = true;
		file2.processed = true;	
     		
		
	   // Capture stdout
	   dockerProcess.stdout.on('data', (data) => {
				console.log(`stdout: ${data}`);
			});

			// Capture stderr
		dockerProcess.stderr.on('data', (data) => {
				console.error(`stderr: ${data}`);
			});

			// Handle process exit
		dockerProcess.on('close', (code) => {
				console.log(`child process exited with code ${code}`);
				  
		 if( code == 0)
			 {   job.output = outFileName
					  console.log('Docker process completed successfully.');
					  resolve("success")
			 }
		else
			 {
					 console.error(`Docker process exited with code ${code}.`);
					 resolve("fail")
			 }
				  
		   });
			
   });

}
const server = http.createServer(requestListener);

server.listen(port, host, () => {
    console.log(`Server is running on http://${host}:${port}`);
});


