"""URL fetching and HTML extraction for the RAG pipeline."""

import logging
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple
from urllib.parse import urlparse, urljoin
import hashlib

import requests
from bs4 import BeautifulSoup
from bs4.element import Tag

from .config import PipelineConfig

logger = logging.getLogger(__name__)


@dataclass
class ExtractedPage:
    """Represents extracted content from a web page."""

    url: str
    title: str
    text: str
    html: str
    section_path: str
    headings: List[str] = field(default_factory=list)
    links: List[str] = field(default_factory=list)
    char_count: int = 0

    def __post_init__(self):
        self.char_count = len(self.text)


class CrawlerError(Exception):
    """Base exception for crawler errors."""

    pass


class HTTPError(CrawlerError):
    """HTTP request failed."""

    def __init__(self, url: str, status_code: int, message: str = ""):
        self.url = url
        self.status_code = status_code
        super().__init__(f"HTTP {status_code} for {url}: {message}")


class TimeoutError(CrawlerError):
    """Request timed out."""

    def __init__(self, url: str, timeout: float):
        self.url = url
        self.timeout = timeout
        super().__init__(f"Timeout ({timeout}s) fetching {url}")


class AuthenticationError(CrawlerError):
    """Authentication required but not provided."""

    def __init__(self, url: str):
        self.url = url
        super().__init__(f"Authentication required for {url}")


class Crawler:
    """Fetches and extracts text content from web pages."""

    # HTML elements to exclude from content extraction
    NON_CONTENT_TAGS = {
        "script", "style", "nav", "header", "footer", "aside", "form",
        "noscript", "iframe", "embed", "object", "svg", "math",
    }

    # CSS selectors for non-content elements to remove
    NON_CONTENT_SELECTORS = [
        "nav", "header", "footer", "aside", ".sidebar", ".navigation",
        ".menu", ".nav", ".breadcrumb", ".pagination", ".advertisement",
        ".ads", ".social-share", ".comments", ".related-posts", ".cookie-banner",
        ".popup", ".modal", ".toast", ".notification",
    ]

    def __init__(self, config: PipelineConfig):
        """Initialize the crawler with configuration."""
        self.config = config
        self.session = requests.Session()
        self.session.headers.update({
            "User-Agent": "RAG-Pipeline/1.0 (+https://github.com/example/rag-pipeline)",
            "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
            "Accept-Language": "en-US,en;q=0.5",
        })

    def fetch_page(self, url: str) -> ExtractedPage:
        """
        Fetch and extract content from a single URL.

        Args:
            url: The URL to fetch

        Returns:
            ExtractedPage with extracted content

        Raises:
            HTTPError: If the HTTP request fails with non-2xx status
            TimeoutError: If the request times out
            AuthenticationError: If authentication is required
        """
        logger.info(f"Fetching: {url}")

        try:
            response = self.session.get(
                url,
                timeout=self.config.request_timeout,
                allow_redirects=True,
            )
            response.raise_for_status()

        except requests.exceptions.HTTPError as e:
            if e.response.status_code == 401:
                raise AuthenticationError(url)
            elif e.response.status_code == 403:
                raise AuthenticationError(url)
            elif e.response.status_code == 404:
                raise HTTPError(url, 404, "Page not found")
            else:
                raise HTTPError(url, e.response.status_code, str(e))

        except requests.exceptions.Timeout:
            raise TimeoutError(url, self.config.request_timeout)

        except requests.exceptions.RequestException as e:
            raise CrawlerError(f"Request failed for {url}: {e}")

        # Extract content
        section_path = self._extract_section_path(url)
        soup = BeautifulSoup(response.text, "html.parser")

        # Remove non-content elements
        self._clean_html(soup)

        # Extract main content
        main_content = self._extract_main_content(soup)

        # Extract headings for metadata
        headings = self._extract_headings(soup)

        # Extract text
        text = self._extract_text(main_content)

        # Extract links
        links = self._extract_links(soup, url)

        # Extract title
        title = self._extract_title(soup)

        logger.info(f"Extracted {len(text)} chars from {url}")

        return ExtractedPage(
            url=url,
            title=title,
            text=text,
            html=response.text,
            section_path=section_path,
            headings=headings,
            links=links,
        )

    def fetch_all(self, urls: List[str]) -> List[ExtractedPage]:
        """
        Fetch multiple URLs, continuing on errors.

        Args:
            urls: List of URLs to fetch

        Returns:
            List of successfully extracted pages
        """
        pages = []
        errors: List[Dict[str, Any]] = []

        for url in urls:
            try:
                page = self.fetch_page(url)
                pages.append(page)
            except Exception as e:
                logger.error(f"Failed to fetch {url}: {e}")
                errors.append({
                    "url": url,
                    "error": str(e),
                    "type": type(e).__name__,
                })

        logger.info(f"Fetched {len(pages)}/{len(urls)} URLs successfully")

        return pages, errors

    def _clean_html(self, soup: BeautifulSoup) -> None:
        """Remove non-content elements from HTML."""
        # Remove non-content tags
        for tag in self.NON_CONTENT_TAGS:
            for element in soup.find_all(tag):
                element.decompose()

        # Remove non-content selectors
        for selector in self.NON_CONTENT_SELECTORS:
            for element in soup.select(selector):
                element.decompose()

        # Remove hidden elements
        for element in soup.find_all(style=True):
            style = element.get("style", "")
            if "display:none" in style or "display: none" in style:
                element.decompose()
            if "visibility:hidden" in style or "visibility: hidden" in style:
                element.decompose()

    def _extract_main_content(self, soup: BeautifulSoup) -> Tag:
        """Extract the main content area from HTML."""
        # Try common content selectors for Docusaurus and similar sites
        content_selectors = [
            "article",
            "[role=main]",
            ".main-content",
            "#main-content",
            ".content",
            "#content",
            ".post-content",
            ".article-content",
            ".markdown-body",
        ]

        for selector in content_selectors:
            element = soup.select_one(selector)
            if element and element.get_text(strip=True):
                return element

        # Fallback to body
        body = soup.find("body")
        if body:
            return body

        return soup

    def _extract_headings(self, soup: BeautifulSoup) -> List[str]:
        """Extract all headings from the page."""
        headings = []
        for tag in soup.find_all(["h1", "h2", "h3", "h4", "h5", "h6"]):
            text = tag.get_text(strip=True)
            if text:
                headings.append(text)
        return headings

    def _extract_text(self, element: Tag) -> str:
        """Extract clean text from an HTML element."""
        # Get text and clean up whitespace
        text = element.get_text(separator="\n", strip=True)

        # Remove extra blank lines
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        text = "\n".join(lines)

        return text

    def _extract_links(self, soup: BeautifulSoup, base_url: str) -> List[str]:
        """Extract all links from the page."""
        links = []
        for a in soup.find_all("a", href=True):
            href = a["href"]
            # Convert relative URLs to absolute
            full_url = urljoin(base_url, href)
            # Only include http(s) links
            if full_url.startswith("http"):
                links.append(full_url)
        return list(set(links))  # Remove duplicates

    def _extract_title(self, soup: BeautifulSoup) -> str:
        """Extract the page title."""
        # Try og:title first
        og_title = soup.find("meta", property="og:title")
        if og_title and og_title.get("content"):
            return og_title["content"]

        # Try title tag
        title_tag = soup.find("title")
        if title_tag:
            return title_tag.get_text(strip=True)

        # Try h1
        h1 = soup.find("h1")
        if h1:
            return h1.get_text(strip=True)

        return ""

    def _extract_section_path(self, url: str) -> str:
        """Extract the section path from URL."""
        parsed = urlparse(url)
        path = parsed.path.strip("/")

        # Remove file extensions and common patterns
        path = re.sub(r"\.(html|htm|md|mdown|markdown)$", "", path)
        path = re.sub(r"/index$", "", path)

        return path

    def generate_url_hash(self, url: str) -> str:
        """Generate SHA-256 hash of URL for deterministic IDs."""
        return hashlib.sha256(url.encode()).hexdigest()[:16]

    def fetch_sitemap(self, sitemap_url: str) -> Tuple[List[str], List[Dict[str, Any]]]:
        """
        Fetch and parse sitemap.xml to get all document URLs.

        Args:
            sitemap_url: URL to the sitemap.xml file

        Returns:
            Tuple of (list of doc URLs, list of errors)
        """
        logger.info(f"Fetching sitemap: {sitemap_url}")

        urls = []
        errors = []

        try:
            response = self.session.get(sitemap_url, timeout=self.config.request_timeout)
            response.raise_for_status()

            # Parse XML
            root = ET.fromstring(response.content)

            # Handle namespaces
            ns = {'sm': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

            # Find all URLs
            for url_elem in root.findall('.//sm:url', ns):
                loc = url_elem.find('sm:loc', ns)
                if loc is not None and loc.text:
                    urls.append(loc.text)

            # Also try without namespace (some sitemaps don't use it)
            if not urls:
                for url_elem in root.findall('.//url'):
                    loc = url_elem.find('loc')
                    if loc is not None and loc.text:
                        urls.append(loc.text)

            logger.info(f"Found {len(urls)} URLs in sitemap")

        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to fetch sitemap: {e}")
            errors.append({
                "url": sitemap_url,
                "error": str(e),
                "type": type(e).__name__,
            })

        return urls, errors

    def filter_doc_urls(self, urls: List[str], base_url: str) -> List[str]:
        """
        Filter URLs to only include documentation pages.

        Args:
            urls: List of URLs from sitemap
            base_url: Base URL of the site

        Returns:
            Filtered list of documentation URLs
        """
        doc_urls = []
        for url in urls:
            # Skip non-docs URLs (like blog, main page, etc.)
            if '/docs/' in url:
                doc_urls.append(url)
            # Also include the main docs page
            elif url.rstrip('/') == base_url.rstrip('/') + '/docs':
                doc_urls.append(url)

        return doc_urls
