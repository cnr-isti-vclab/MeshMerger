# Use the pre-built TBB image as the base
FROM mesh_merger_dependencies_compressed as build

WORKDIR /app

# Install the npm package for working with PLY files
RUN npm install ply

# Copy your project into the Docker image
COPY ./mesh_merger /app/

# Run dos2unix on any necessary files (e.g., config.txt)
RUN dos2unix /app/jobs/config.txt

# Set permissions for all PLY files in the image
RUN find /app -type f -name "*.ply" -exec chmod 644 {} +

# Debug: List the directory structure
RUN ls -la /app/ && ls -la /app/Registration/

# Build your project
RUN make -C /app

# Cleanup steps
RUN rm -rf /var/cache/apk/* /tmp/* /var/tmp/*

# Set the working directory for the CMD
WORKDIR /app/jobs

# Start both Node.js script and charity executable using PM2
CMD ["pm2-runtime", "start", "./meshmerger_ply.js"]
