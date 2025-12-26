/**
 * Search component types.
 */

export interface SearchResult {
  chunk_id: string;
  text: string;
  module_id: string;
  chapter_id: string;
  section_heading: string;
  score: number;
  highlight: string | null;
}

export interface SearchState {
  isOpen: boolean;
  query: string;
  results: SearchResult[];
  isLoading: boolean;
  error: string | null;
}
